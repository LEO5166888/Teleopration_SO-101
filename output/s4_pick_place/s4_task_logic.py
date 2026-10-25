"""Pure success/reset logic for the fixed S-4 pick-and-place task."""

import math


def vector_norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def quaternion_xyzw_to_rotation_vector(q_rel):
    """Convert an App ``[x, y, z, w]`` quaternion to a rotation vector.

    The returned vector direction is the rotation axis and its magnitude is the
    shortest rotation angle in radians.  Keeping this helper free of Isaac
    imports lets the orientation math be unit-tested on the Mac.
    """
    if len(q_rel) != 4 or not all(math.isfinite(value) for value in q_rel):
        raise ValueError("q_rel must contain four finite values")
    x, y, z, w = (float(value) for value in q_rel)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        raise ValueError("q_rel norm is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # q and -q encode the same orientation; choose the shorter-angle form.
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_norm_value = math.sqrt(x * x + y * y + z * z)
    if vector_norm_value < 1.0e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm_value, max(w, 0.0))
    scale = angle / vector_norm_value
    return (x * scale, y * scale, z * scale)


class PitchOnlyClutch:
    """Latch pitch control until release, holding all other joint targets.

    Input q is already shaped by the phone App. This helper adds no gain or
    smoothing. The caller must bypass position IK when a tuple is returned.
    """

    def __init__(self, enter_deg=1.0, max_flex_deg=35.0):
        if not all(math.isfinite(v) and v > 0 for v in (enter_deg, max_flex_deg)):
            raise ValueError("pitch thresholds must be finite and positive")
        self.enter_rad = math.radians(enter_deg)
        self.max_flex_rad = math.radians(max_flex_deg)
        self.reset()

    def reset(self):
        self.active = False
        self._targets = None
        self.pitch_rad = 0.0
        self.clamped = False

    def update(self, q_rel, joint_targets, *, flex_index, start_flex,
               flex_sign, lower, upper, max_step, control_mode=None):
        # None retains the legacy helper behavior for regression tests only.
        # Runtime always passes the operator's explicit choice.
        if control_mode not in (None, "translation", "wrist"):
            raise ValueError("unknown pose control mode")
        if control_mode == "translation":
            return None
        pitch = quaternion_xyzw_to_rotation_vector(q_rel)[0]
        targets = tuple(float(v) for v in joint_targets)
        if (not targets or not all(math.isfinite(v) for v in targets)
                or not isinstance(flex_index, int)
                or not 0 <= flex_index < len(targets)):
            raise ValueError("joint targets and flex index must be valid")
        if (not all(math.isfinite(v) for v in
                    (start_flex, flex_sign, lower, upper, max_step))
                or lower > upper or max_step <= 0 or flex_sign not in (-1, 1)
                or not lower <= targets[flex_index] <= upper):
            raise ValueError("invalid wrist bounds, sign or step")
        if control_mode != "wrist" and not self.active and abs(pitch) < self.enter_rad:
            return None
        current = self._targets if self.active else targets
        relative = max(-self.max_flex_rad, min(self.max_flex_rad, flex_sign * pitch))
        desired = max(lower, min(upper, start_flex + relative))
        result = list(current)
        result[flex_index] += max(-max_step, min(max_step, desired - current[flex_index]))
        self._targets = tuple(result)
        self.active = True
        self.pitch_rad = pitch
        self.clamped = relative != flex_sign * pitch or desired != start_flex + relative
        return self._targets


def _soft_deadband(value, deadband):
    """Drop small motion without introducing a jump at the threshold."""
    if not math.isfinite(deadband) or deadband < 0.0:
        raise ValueError("deadband must be finite and non-negative")
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0
    return math.copysign(magnitude - deadband, value)


def wrist_joint_deltas_rad(
    q_rel,
    *,
    gain,
    deadband_deg,
    smoothing,
    previous_flex_rad,
    previous_roll_rad,
    flex_sign=1.0,
    roll_sign=-1.0,
    max_flex_deg=35.0,
    max_roll_deg=60.0,
):
    """Map phone pitch/roll to wrist joints; phone yaw is ignored."""
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("gain must be a finite non-negative number")
    if not math.isfinite(smoothing) or not 0.0 <= smoothing <= 0.95:
        raise ValueError("smoothing must be finite and within [0, 0.95]")
    if not all(
        math.isfinite(value) for value in (previous_flex_rad, previous_roll_rad)
    ):
        raise ValueError("previous wrist deltas must be finite")
    if not all(math.isfinite(value) for value in (flex_sign, roll_sign)):
        raise ValueError("signs must be finite")
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (deadband_deg, max_flex_deg, max_roll_deg)
    ):
        raise ValueError("limits and deadband must be finite and non-negative")

    x_rad, y_rad, _yaw_rad = quaternion_xyzw_to_rotation_vector(q_rel)
    deadband_rad = math.radians(deadband_deg)
    flex = _soft_deadband(flex_sign * gain * x_rad, deadband_rad)
    roll = _soft_deadband(roll_sign * gain * y_rad, deadband_rad)
    flex = smoothing * previous_flex_rad + (1.0 - smoothing) * flex
    roll = smoothing * previous_roll_rad + (1.0 - smoothing) * roll
    flex = max(-math.radians(max_flex_deg), min(math.radians(max_flex_deg), flex))
    roll = max(-math.radians(max_roll_deg), min(math.radians(max_roll_deg), roll))
    return (flex, roll)


def map_webxr_translation_to_base(p_rel_m, gain=1.0):
    """Map XRViewer translation into the current S4 robot base frame.

    WebXR ``local`` uses x=phone right, y=up, z=back toward the user.  The S4
    operator view uses base x positive toward screen-left, base y negative
    forward onto the table, and base z positive up.  Live XRViewer testing on
    the target iPhone showed that all three first-pass axes were inverted from
    the intended operator motion.  The empirically calibrated mapping is:

    ``base = gain * [-webxr_x, -webxr_z, webxr_y]``.

    Keeping this mapping in the pure-logic module makes the sign convention
    explicit, unit-testable, and easy to recalibrate after the first live run.
    """
    if len(p_rel_m) != 3 or not all(math.isfinite(value) for value in p_rel_m):
        raise ValueError("p_rel_m must contain three finite values")
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("gain must be a finite non-negative number")
    x_right, y_up, z_back = (float(value) for value in p_rel_m)
    return (-gain * x_right, -gain * z_back, gain * y_up)


def map_calibrated_webxr_translation_to_base(p_rel_m, gain=1.0):
    """Candidate base mapping from Leo's labelled 2026-10-25 trials.

    Captured phone setup: left +XR X, forward -XR Y, up -XR Z.
    Robot task convention: left +base X, forward -base Y, up +base Z.
    This empirical mapping is NOT a universal WebXR axis convention and does
    not calibrate arbitrary reference-space resets or phone headings.
    Off-axis measured motion is preserved; this function does not filter it.
    Keep the legacy helper above for the unused tool-heading controller.
    """
    if len(p_rel_m) != 3 or not all(math.isfinite(value) for value in p_rel_m):
        raise ValueError("p_rel_m must contain three finite values")
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("gain must be a finite non-negative number")
    xr_x, xr_y, xr_z = (float(value) for value in p_rel_m)
    return (gain * xr_x, gain * xr_y, -gain * xr_z)


def map_webxr_translation_to_tool_heading(p_rel_m, tool_forward_xy, gain=1.0):
    """Map phone translation into a clutch-time tool-heading frame.

    Horizontal forward follows the projected wrist-to-gripper direction at
    ``pose_start``.  Horizontal right is perpendicular to that direction.
    Vertical remains the robot base/world z axis, so tilting the gripper never
    turns an operator "up" motion into motion toward the table.
    """
    if len(tool_forward_xy) != 2 or not all(
        math.isfinite(value) for value in tool_forward_xy
    ):
        raise ValueError("tool_forward_xy must contain two finite values")
    forward_x, forward_y = (float(value) for value in tool_forward_xy)
    heading_norm = math.hypot(forward_x, forward_y)
    if heading_norm < 1.0e-9:
        raise ValueError("tool_forward_xy norm is zero")
    forward_x /= heading_norm
    forward_y /= heading_norm
    # forward x world-up points to tool-right in the horizontal plane.
    right_x, right_y = forward_y, -forward_x
    lateral, longitudinal, vertical = map_webxr_translation_to_base(
        p_rel_m,
        gain,
    )
    return (
        lateral * right_x + longitudinal * forward_x,
        lateral * right_y + longitudinal * forward_y,
        vertical,
    )


def filter_webxr_translation(
    p_rel_m,
    previous_filtered_m,
    deadband_m=0.004,
    smoothing=0.75,
):
    """Suppress stationary WebXR drift before mapping it to robot motion.

    The deadband is continuous and independent per axis: a 6 mm input with a
    4 mm deadband becomes 2 mm instead of jumping directly to 6 mm.
    ``smoothing`` is the fraction of the previous filtered value retained.
    """
    if len(p_rel_m) != 3 or len(previous_filtered_m) != 3:
        raise ValueError("translations must contain three values")
    if not all(
        math.isfinite(float(value))
        for value in (*p_rel_m, *previous_filtered_m)
    ):
        raise ValueError("translations must contain finite values")
    if not math.isfinite(deadband_m) or deadband_m < 0.0:
        raise ValueError("deadband_m must be finite and non-negative")
    if not math.isfinite(smoothing) or not 0.0 <= smoothing < 1.0:
        raise ValueError("smoothing must be finite and within [0, 1)")
    shaped = []
    for value in p_rel_m:
        magnitude = max(0.0, abs(float(value)) - deadband_m)
        shaped.append(math.copysign(magnitude, value) if magnitude else 0.0)
    return tuple(
        smoothing * float(previous) + (1.0 - smoothing) * current
        for previous, current in zip(previous_filtered_m, shaped)
    )


def advance_tracking_safety(error_m, limit_m, streak, confirm_steps):
    """Require a persistent tracking error before asking for a hard stop."""
    if confirm_steps < 1:
        raise ValueError("confirm_steps must be positive")
    if not math.isfinite(limit_m) or limit_m <= 0.0:
        raise ValueError("limit_m must be a positive finite number")
    if not isinstance(streak, int) or streak < 0:
        raise ValueError("streak must be a non-negative integer")
    if not math.isfinite(error_m) or error_m > limit_m:
        streak += 1
    else:
        streak = 0
    return (streak >= confirm_steps, streak)


def inside_goal(object_pos, goal_pos, half_extents):
    """Return True when the object center is inside the fixed goal box."""
    if not all(len(values) == 3 for values in (object_pos, goal_pos, half_extents)):
        raise ValueError("object_pos, goal_pos, and half_extents must have length 3")
    return all(
        abs(float(position) - float(center)) <= float(half_extent) + 1.0e-12
        for position, center, half_extent in zip(object_pos, goal_pos, half_extents)
    )


def success_sample(
    object_pos,
    goal_pos,
    half_extents,
    linear_velocity,
    angular_velocity,
    gripper_open,
    max_linear_speed,
    max_angular_speed,
):
    """Evaluate one physics sample of the S-4 success condition."""
    return (
        bool(gripper_open)
        and inside_goal(object_pos, goal_pos, half_extents)
        and vector_norm(linear_velocity) <= max_linear_speed
        and vector_norm(angular_velocity) <= max_angular_speed
    )


class StableSuccess:
    """Latch success after the condition remains true for consecutive samples."""

    def __init__(self, required_samples):
        if required_samples < 1:
            raise ValueError("required_samples must be positive")
        self.required_samples = required_samples
        self.count = 0
        self.latched = False

    def update(self, sample_is_success):
        if self.latched:
            return True
        self.count = self.count + 1 if sample_is_success else 0
        self.latched = self.count >= self.required_samples
        return self.latched

    def reset(self):
        self.count = 0
        self.latched = False
