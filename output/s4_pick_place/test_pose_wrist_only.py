"""Wrist-only clutch requirements. Synthetic q; live pitch axis still needs QA."""
import math
import unittest

import s4_task_logic as logic
from test_phone_translation_20261025 import TRIALS


def pitch_q(degrees):
    half = math.radians(degrees) / 2
    return (math.sin(half), 0, 0, math.cos(half))


class WristOnlyTests(unittest.TestCase):
    def setUp(self):
        self.gate = logic.PitchOnlyClutch(enter_deg=1.0, max_flex_deg=35.0)
        self.joints = (0.1, -1.0, 1.2, 0.7, 0.2, 0.3)

    def update(self, q, joints=None, sign=1):
        return self.gate.update(
            q, self.joints if joints is None else joints,
            flex_index=3, start_flex=0.7, flex_sign=sign,
            lower=-1.65, upper=1.65, max_step=0.01,
        )

    def test_small_pitch_does_not_interrupt_translation(self):
        for angle in (0, 0.01, 0.5, -0.99):
            self.assertIsNone(self.update(pitch_q(angle)))
        self.assertFalse(self.gate.active)

    def test_six_calibrated_translation_inputs_remain_available(self):
        for _, label, raw, axis, sign in TRIALS:
            with self.subTest(label=label):
                self.assertIsNone(self.update(pitch_q(0)))
                result = logic.map_calibrated_webxr_translation_to_base(raw)
                self.assertGreater(result[axis] * sign, 0)

    def test_only_wrist_flex_changes_and_step_is_bounded(self):
        for angle in (10, -10):
            self.gate.reset()
            target = self.update(pitch_q(angle))
            for i in (0, 1, 2, 4, 5):
                self.assertEqual(target[i], self.joints[i])
            self.assertAlmostEqual(abs(target[3] - self.joints[3]), 0.01)
            self.assertGreater((target[3] - self.joints[3]) * angle, 0)

    def test_freeze_is_latched_and_does_not_follow_new_arm_targets(self):
        self.update(pitch_q(10))
        target = self.update(pitch_q(0), joints=(0.4, -0.8, 1.4, 0.71, 0.9, 0.8))
        self.assertTrue(self.gate.active)
        for i in (0, 1, 2, 4, 5):
            self.assertEqual(target[i], self.joints[i])
        self.assertAlmostEqual(target[3], 0.7)
        self.gate.reset()
        self.assertIsNone(self.update(pitch_q(0)))

    def test_pure_roll_and_yaw_do_not_activate_pitch_mode(self):
        h = math.radians(20) / 2
        for q in ((0, math.sin(h), 0, math.cos(h)),
                  (0, 0, math.sin(h), math.cos(h))):
            self.assertIsNone(self.update(q))

    def test_sign_switch_only_reverses_flex(self):
        target = self.update(pitch_q(10), sign=-1)
        self.assertAlmostEqual(target[3], self.joints[3] - 0.01)
        for i in (0, 1, 2, 4, 5):
            self.assertEqual(target[i], self.joints[i])

    def test_equivalent_quaternion_sign_has_identical_result(self):
        q = pitch_q(-12)
        expected = self.update(q)
        self.gate.reset()
        self.assertEqual(expected, self.update(tuple(-v for v in q)))

    def test_joint_limit_and_relative_limit_are_respected(self):
        for _ in range(200):
            target = self.update(pitch_q(80))
        self.assertAlmostEqual(target[3], 0.7 + math.radians(35))
        self.gate.reset()
        for _ in range(200):
            target = self.gate.update(
                pitch_q(80), self.joints, flex_index=3, start_flex=0.7,
                flex_sign=1, lower=-0.8, upper=0.75, max_step=0.01,
            )
        self.assertAlmostEqual(target[3], 0.75)

    def test_invalid_input_is_rejected_before_state_changes(self):
        with self.assertRaises(ValueError):
            self.update((float('nan'), 0, 0, 1))
        self.assertFalse(self.gate.active)
        with self.assertRaises(ValueError):
            self.update(pitch_q(10), joints=(0, float('inf'), 0, 0, 0, 0))
        self.assertFalse(self.gate.active)


if __name__ == '__main__':
    unittest.main()
