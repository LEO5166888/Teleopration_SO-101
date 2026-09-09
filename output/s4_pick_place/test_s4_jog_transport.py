import concurrent.futures
import time
import unittest

from a3_transport import (
    Bridge,
    validate_empty,
    validate_gripper_jog_start,
    validate_jog_start,
)


class JogValidationTests(unittest.TestCase):
    def test_valid_jog_start(self):
        self.assertIsNone(validate_jog_start({
            "axis": "z",
            "dir": 1,
            "step": "fine",
            "frame": "base",
        }))

    def test_invalid_jog_start(self):
        self.assertIsNotNone(validate_jog_start({
            "axis": "rx",
            "dir": 1,
            "step": "fine",
            "frame": "base",
        }))
        self.assertIsNotNone(validate_empty({"extra": True}, "jog_stop"))

    def test_owner_can_refresh_and_stop_active_jog(self):
        bridge = Bridge()
        owner = object()
        request = {
            "owner": owner,
            "message": {"type": "jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request

        before = time.monotonic()
        self.assertTrue(bridge.refresh_hold(owner))
        self.assertGreater(request["deadline"], before)
        self.assertEqual(
            bridge.request_jog_stop(owner),
            ("ok", "jog_stop_requested"),
        )
        self.assertTrue(bridge.jog_stop_event.is_set())

    def test_other_owner_cannot_stop_jog(self):
        bridge = Bridge()
        request = {
            "owner": object(),
            "message": {"type": "jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request
        self.assertEqual(
            bridge.request_jog_stop(object()),
            ("rejected", "jog_not_active"),
        )

    def test_valid_gripper_jog_start(self):
        self.assertIsNone(validate_gripper_jog_start({
            "action": "close",
            "step": "fine",
        }))
        self.assertIsNotNone(validate_gripper_jog_start({
            "action": "hold",
            "step": "fine",
        }))

    def test_owner_can_refresh_and_stop_gripper_jog(self):
        bridge = Bridge()
        owner = object()
        request = {
            "owner": owner,
            "message": {"type": "gripper_jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request

        before = time.monotonic()
        self.assertTrue(bridge.refresh_hold(owner))
        self.assertGreater(request["deadline"], before)
        self.assertEqual(
            bridge.request_gripper_jog_stop(owner),
            ("ok", "gripper_jog_stop_requested"),
        )
        self.assertTrue(bridge.gripper_jog_stop_event.is_set())

    def test_other_owner_cannot_stop_gripper_jog(self):
        bridge = Bridge()
        request = {
            "owner": object(),
            "message": {"type": "gripper_jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request
        self.assertEqual(
            bridge.request_gripper_jog_stop(object()),
            ("rejected", "gripper_jog_not_active"),
        )

    def test_owner_can_refresh_and_stop_joint_jog(self):
        bridge = Bridge()
        owner = object()
        request = {
            "owner": owner,
            "message": {"type": "joint_jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request

        before = time.monotonic()
        self.assertTrue(bridge.refresh_hold(owner))
        self.assertGreater(request["deadline"], before)
        self.assertEqual(
            bridge.request_joint_jog_stop(owner),
            ("ok", "joint_jog_stop_requested"),
        )
        self.assertTrue(bridge.joint_jog_stop_event.is_set())

    def test_other_owner_cannot_stop_joint_jog(self):
        bridge = Bridge()
        request = {
            "owner": object(),
            "message": {"type": "joint_jog_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
        }
        bridge.pending = request
        self.assertEqual(
            bridge.request_joint_jog_stop(object()),
            ("rejected", "joint_jog_not_active"),
        )


if __name__ == "__main__":
    unittest.main()
