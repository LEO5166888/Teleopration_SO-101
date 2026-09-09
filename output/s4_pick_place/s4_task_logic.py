"""Pure success/reset logic for the fixed S-4 pick-and-place task."""

import math


def vector_norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


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
