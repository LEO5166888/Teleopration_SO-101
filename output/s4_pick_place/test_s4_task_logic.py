import unittest

from s4_task_logic import StableSuccess, inside_goal, success_sample


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


if __name__ == "__main__":
    unittest.main()
