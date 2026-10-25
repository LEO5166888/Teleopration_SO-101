import math
import unittest

from s4_task_logic import (
    StableSuccess,
    advance_tracking_safety,
    filter_webxr_translation,
    inside_goal,
    map_webxr_translation_to_base,
    map_webxr_translation_to_tool_heading,
    quaternion_xyzw_to_rotation_vector,
    success_sample,
    wrist_joint_deltas_rad,
)


class S4TaskLogicTests(unittest.TestCase):
    def setUp(self):
        self.object_pos = (0.075, -0.140, 0.103)
        self.goal_pos = (0.075, -0.140, 0.103)
        self.half_extents = (0.025, 0.025, 0.020)

    def test_inside_goal_includes_boundary(self):
        self.assertTrue(
            inside_goal((0.100, -0.165, 0.123), self.goal_pos, self.half_extents)
        )
        self.assertFalse(
            inside_goal((0.101, -0.140, 0.103), self.goal_pos, self.half_extents)
        )

    def test_success_requires_release_and_low_velocity(self):
        common = dict(
            object_pos=self.object_pos,
            goal_pos=self.goal_pos,
            half_extents=self.half_extents,
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
            max_linear_speed=0.02,
            max_angular_speed=0.50,
        )
        self.assertTrue(success_sample(gripper_open=True, **common))
        self.assertFalse(success_sample(gripper_open=False, **common))
        self.assertFalse(
            success_sample(
                gripper_open=True,
                **{**common, "linear_velocity": (0.03, 0.0, 0.0)},
            )
        )

    def test_stability_must_be_consecutive_and_latches(self):
        latch = StableSuccess(required_samples=3)
        self.assertFalse(latch.update(True))
        self.assertFalse(latch.update(False))
        self.assertFalse(latch.update(True))
        self.assertFalse(latch.update(True))
        self.assertTrue(latch.update(True))
        self.assertTrue(latch.update(False))
        latch.reset()
        self.assertFalse(latch.latched)
        self.assertEqual(latch.count, 0)

    def test_quaternion_rotation_vector_identity_and_x_rotation(self):
        self.assertEqual(
            quaternion_xyzw_to_rotation_vector((0.0, 0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )
        angle = math.radians(30.0)
        result = quaternion_xyzw_to_rotation_vector((
            math.sin(angle / 2.0),
            0.0,
            0.0,
            math.cos(angle / 2.0),
        ))
        self.assertAlmostEqual(result[0], angle)
        self.assertAlmostEqual(result[1], 0.0)
        self.assertAlmostEqual(result[2], 0.0)

    def test_quaternion_sign_equivalence_and_zero_rejected(self):
        positive = quaternion_xyzw_to_rotation_vector((0.0, 0.5, 0.0, 0.8660254))
        negative = quaternion_xyzw_to_rotation_vector((0.0, -0.5, 0.0, -0.8660254))
        self.assertEqual(positive, negative)
        with self.assertRaises(ValueError):
            quaternion_xyzw_to_rotation_vector((0.0, 0.0, 0.0, 0.0))

    def test_wrist_mapping_ignores_yaw(self):
        angle = math.radians(30.0)
        result = wrist_joint_deltas_rad(
            (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)),
            gain=1.0,
            deadband_deg=2.5,
            smoothing=0.0,
            previous_flex_rad=0.0,
            previous_roll_rad=0.0,
        )
        self.assertEqual(result, (0.0, 0.0))

    def test_wrist_mapping_applies_soft_deadband(self):
        angle = math.radians(10.0)
        flex, roll = wrist_joint_deltas_rad(
            (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)),
            gain=1.0,
            deadband_deg=5.0,
            smoothing=0.0,
            previous_flex_rad=0.0,
            previous_roll_rad=0.0,
        )
        self.assertAlmostEqual(flex, math.radians(5.0))
        self.assertAlmostEqual(roll, 0.0)

    def test_webxr_translation_mapping(self):
        self.assertEqual(
            map_webxr_translation_to_base((0.10, 0.20, 0.30), gain=0.5),
            (-0.05, -0.15, 0.10),
        )
        # The signs below are the live XRViewer calibration for this iPhone.
        self.assertEqual(
            map_webxr_translation_to_base((0.0, 0.0, -0.10)),
            (0.0, 0.10, 0.0),
        )

    def test_webxr_translation_filter_rejects_stationary_drift(self):
        self.assertEqual(
            filter_webxr_translation(
                (0.003, -0.004, 0.001),
                (0.0, 0.0, 0.0),
                deadband_m=0.004,
                smoothing=0.0,
            ),
            (0.0, 0.0, 0.0),
        )

    def test_webxr_translation_filter_is_continuous_and_smoothed(self):
        filtered = filter_webxr_translation(
            (0.010, -0.006, 0.004),
            (0.002, 0.0, 0.0),
            deadband_m=0.004,
            smoothing=0.75,
        )
        self.assertAlmostEqual(filtered[0], 0.003)
        self.assertAlmostEqual(filtered[1], -0.0005)
        self.assertAlmostEqual(filtered[2], 0.0)

    def test_webxr_translation_filter_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            filter_webxr_translation((0.0, 0.0), (0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            filter_webxr_translation(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                smoothing=1.0,
            )

    def test_tracking_safety_requires_consecutive_errors(self):
        streak = 0
        for _ in range(14):
            tripped, streak = advance_tracking_safety(0.11, 0.10, streak, 15)
            self.assertFalse(tripped)
        tripped, streak = advance_tracking_safety(0.11, 0.10, streak, 15)
        self.assertTrue(tripped)

    def test_tracking_safety_resets_after_recovery(self):
        self.assertEqual(
            advance_tracking_safety(0.01, 0.10, 7, 15),
            (False, 0),
        )

    def test_webxr_translation_mapping_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            map_webxr_translation_to_base((0.0, 0.0))
        with self.assertRaises(ValueError):
            map_webxr_translation_to_base((0.0, 0.0, 0.0), gain=-1.0)

    def test_tool_heading_mapping_keeps_world_vertical(self):
        forward_xy = (0.0, -1.0)
        self.assertEqual(
            map_webxr_translation_to_tool_heading(
                (0.10, 0.20, -0.30),
                forward_xy,
            ),
            (0.10, -0.30, 0.20),
        )

    def test_tool_heading_mapping_normalizes_heading(self):
        self.assertEqual(
            map_webxr_translation_to_tool_heading(
                (0.10, 0.0, 0.0),
                (0.0, -5.0),
            ),
            (0.10, 0.0, 0.0),
        )
        with self.assertRaises(ValueError):
            map_webxr_translation_to_tool_heading(
                (0.0, 0.0, 0.0),
                (0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main()
