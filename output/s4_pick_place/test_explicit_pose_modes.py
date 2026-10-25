import math
import unittest

from a3_transport import validate_pose_start
from s4_task_logic import PitchOnlyClutch


class ExplicitPoseModeTests(unittest.TestCase):
    def test_protocol_modes_and_legacy_default(self):
        payload = dict(frame="webxr_local", rate_hz=30, gain=0.7,
                       deadband_deg=2.5, smoothing=0.3)
        self.assertIsNone(validate_pose_start(payload))
        for mode in ("translation", "wrist"):
            self.assertIsNone(validate_pose_start(dict(payload, control_mode=mode)))
        for mode in ("auto", "", None, 1, [], {}):
            self.assertIsNotNone(validate_pose_start(dict(payload, control_mode=mode)))
        self.assertIsNotNone(validate_pose_start(dict(payload, extra=True)))

    def update(self, clutch, mode, pitch=0, joints=(0.1, -1, 1.2, 0.7, 0.2, 0.3)):
        half = math.radians(pitch) / 2
        return clutch.update(
            (math.sin(half), 0, 0, math.cos(half)), joints,
            flex_index=3, start_flex=joints[3], flex_sign=1,
            lower=-1.65, upper=1.65, max_step=0.01, control_mode=mode,
        )

    def test_translation_never_switches_to_wrist_on_tilt(self):
        clutch = PitchOnlyClutch()
        for pitch in (0, -1.03, -4.55, 15, -35, 80):
            self.assertIsNone(self.update(clutch, "translation", pitch))
            self.assertFalse(clutch.active)

    def test_wrist_holds_arm_from_first_neutral_sample(self):
        clutch = PitchOnlyClutch()
        initial = (0.1, -1, 1.2, 0.7, 0.2, 0.3)
        self.assertEqual(self.update(clutch, "wrist"), initial)
        target = self.update(clutch, "wrist", 10, (0.8, -0.5, 1.3, 0.7, 0.4, 0.9))
        for index in (0, 1, 2, 4, 5):
            self.assertEqual(target[index], initial[index])
        self.assertGreater(target[3], initial[3])

    def test_next_press_resets_wrist_hold_and_allows_translation(self):
        clutch = PitchOnlyClutch()
        self.update(clutch, "wrist", 10)
        clutch.reset()
        self.assertIsNone(self.update(clutch, "translation", -15))
        self.assertFalse(clutch.active)
        clutch.reset()
        new_pose = (0.4, -0.5, 0.8, 0.9, 0.2, 0.3)
        self.assertEqual(self.update(clutch, "wrist", joints=new_pose), new_pose)


if __name__ == "__main__":
    unittest.main()
