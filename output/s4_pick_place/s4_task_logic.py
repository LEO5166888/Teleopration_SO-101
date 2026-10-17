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
