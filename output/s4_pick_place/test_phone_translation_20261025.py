"""Direction contract from Leo's labelled XRViewer trials, 2026-10-25.

Robot task axes: +X left (latest visual report), -Y forward toward the
table/object, +Z up. These expectations are independent of the old mapping.
Calibration applies only to this captured XR session / phone setup.
"""

import unittest

from s4_task_logic import (
    map_calibrated_webxr_translation_to_base as map_webxr_translation_to_base,
)


# Raw end-of-hold inputs, not the robot's already-transformed displacements.
TRIALS = (
    (396, "right", (-0.0239, 0.0009, 0.0016), 0, -1),
    (968, "left", (0.0290, -0.0092, -0.0017), 0, 1),
    (1209, "forward", (-0.0068, -0.0360, 0.0002), 1, -1),
    (1808, "backward", (-0.0003, 0.0483, -0.0036), 1, 1),
    (1718, "up", (-0.0079, 0.0021, -0.0347), 2, 1),
    (1761, "down", (-0.0093, 0.0063, 0.0517), 2, -1),
)


class PhoneTranslationCalibrationTests(unittest.TestCase):
    def test_recorded_trials_have_expected_dominant_direction(self):
        for seq, label, raw, expected_axis, expected_sign in TRIALS:
            with self.subTest(seq=seq, motion=label):
                result = map_webxr_translation_to_base(raw)
                dominant_axis = max(range(3), key=lambda i: abs(result[i]))
                self.assertEqual(dominant_axis, expected_axis)
                self.assertGreater(result[expected_axis] * expected_sign, 0)

    def test_isolated_axes_do_not_create_cross_axis_motion(self):
        # Idealized single-axis inputs derived from the measured dominant axes.
        cases = (
            ((-0.02, 0, 0), (-0.02, 0, 0)),  # right
            ((0.02, 0, 0), (0.02, 0, 0)),    # left
            ((0, -0.02, 0), (0, -0.02, 0)),  # forward
            ((0, 0.02, 0), (0, 0.02, 0)),    # backward
            ((0, 0, -0.02), (0, 0, 0.02)),   # up
            ((0, 0, 0.02), (0, 0, -0.02)),   # down
        )
        for raw, expected in cases:
            with self.subTest(raw=raw):
                self.assertEqual(map_webxr_translation_to_base(raw), expected)

    def test_zero_input_and_zero_gain(self):
        self.assertEqual(map_webxr_translation_to_base((0, 0, 0)), (0, 0, 0))
        self.assertEqual(
            map_webxr_translation_to_base((0.02, -0.03, 0.04), gain=0),
            (0, 0, 0),
        )

    def test_gain_scales_displacement_without_changing_direction(self):
        self.assertEqual(
            map_webxr_translation_to_base((0.02, -0.04, -0.06), gain=0.5),
            (0.01, -0.02, 0.03),
        )

    def test_invalid_inputs_are_rejected(self):
        for raw in ((0, 0), (0, 0, float("nan")), (float("inf"), 0, 0)):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                map_webxr_translation_to_base(raw)
        for gain in (-1, float("nan"), float("inf")):
            with self.subTest(gain=gain), self.assertRaises(ValueError):
                map_webxr_translation_to_base((0, 0, 0), gain=gain)


if __name__ == "__main__":
    unittest.main()
