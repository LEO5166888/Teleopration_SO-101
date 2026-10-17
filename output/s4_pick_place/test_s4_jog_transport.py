import asyncio
import concurrent.futures
import json
import time
import unittest

from a3_transport import (
    Bridge,
    envelope,
    validate_empty,
    validate_envelope,
    validate_gripper_jog_start,
    validate_jog_start,
    validate_orient_start,
    validate_orient_update,
    validate_pose_start,
    validate_pose_update,
)


class JogValidationTests(unittest.TestCase):
    def test_protocol_v0_and_v1_are_supported(self):
        self.assertIsNone(validate_envelope(envelope("hello", 1, {}, version=0)))
        self.assertIsNone(validate_envelope(envelope("hello", 1, {}, version=1)))
        self.assertEqual(
            validate_envelope(envelope("hello", 1, {}, version=2))[1],
            "unsupported_version",
        )

    def test_bridge_replies_using_client_version(self):
        class FakeWebSocket:
            def __init__(self):
                self.messages = []

            async def send(self, text):
                self.messages.append(json.loads(text))

        bridge = Bridge()
        ws = FakeWebSocket()
        bridge.connection_versions[ws] = 1
        asyncio.run(bridge.send(ws, "welcome", 3, {"session": "test"}))
        self.assertEqual(ws.messages[0]["v"], 1)

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

    def test_valid_orientation_messages(self):
        self.assertIsNone(validate_orient_start({
            "frame": "phone_body",
            "rate_hz": 30,
            "gain": 0.7,
            "deadband_deg": 2.5,
            "smoothing": 0.3,
        }))
        self.assertIsNone(validate_orient_update({
            "q_rel": [0.0, 0.0, 0.0, 1.0],
        }))

    def test_invalid_orientation_messages(self):
        invalid_start = {
            "frame": "world",
            "rate_hz": 30,
            "gain": 0.7,
            "deadband_deg": 2.5,
            "smoothing": 0.3,
        }
        self.assertEqual(
            validate_orient_start(invalid_start)[0],
            "invalid_orient_frame",
        )
        self.assertEqual(
            validate_orient_update({"q_rel": [0.0, 0.0, 0.0, 0.0]})[0],
            "invalid_orient_quaternion",
        )

    def test_orientation_uses_its_own_keepalive_and_stop(self):
        bridge = Bridge()
        owner = object()
        request = {
            "owner": owner,
            "message": {"type": "orient_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
            "q_rel": [0.0, 0.0, 0.0, 1.0],
        }
        bridge.pending = request

        self.assertFalse(bridge.refresh_hold(owner))
        before = time.monotonic()
        q_rel = [0.0, 0.0, 0.2588190451, 0.9659258263]
        self.assertTrue(bridge.refresh_orient(owner, q_rel))
        self.assertGreater(request["deadline"], before)
        self.assertEqual(bridge.orient_snapshot(request), q_rel)
        self.assertEqual(
            bridge.request_orient_stop(owner),
            ("ok", "orient_stop_requested"),
        )
        self.assertTrue(bridge.orient_stop_event.is_set())

    def test_other_owner_cannot_update_or_stop_orientation(self):
        bridge = Bridge()
        request = {
            "owner": object(),
            "message": {"type": "orient_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
            "q_rel": [0.0, 0.0, 0.0, 1.0],
        }
        bridge.pending = request
        other_owner = object()
        self.assertFalse(
            bridge.refresh_orient(other_owner, [0.0, 0.0, 0.0, 1.0])
        )
        self.assertEqual(
            bridge.request_orient_stop(other_owner),
            ("rejected", "orient_not_active"),
        )

    def test_valid_pose_messages(self):
        self.assertIsNone(validate_pose_start({
            "frame": "webxr_local",
            "rate_hz": 30,
            "gain": 0.7,
            "deadband_deg": 2.5,
            "smoothing": 0.3,
        }))
        self.assertIsNone(validate_pose_update({
            "p_rel_m": [0.1, 0.0, -0.2],
            "q_rel": [0.0, 0.0, 0.0, 1.0],
            "gripper_delta": 0.5,
        }))

    def test_invalid_pose_messages(self):
        valid = {
            "p_rel_m": [0.1, 0.0, -0.2],
            "q_rel": [0.0, 0.0, 0.0, 1.0],
            "gripper_delta": 0.5,
        }
        self.assertEqual(
            validate_pose_update({**valid, "p_rel_m": [0.0, 0.0]})[0],
            "invalid_pose_position",
        )
        self.assertEqual(
            validate_pose_update({**valid, "q_rel": [0.0, 0.0, 0.0, 0.0]})[0],
            "invalid_pose_quaternion",
        )
        self.assertEqual(
            validate_pose_update({**valid, "gripper_delta": 1.1})[0],
            "invalid_pose_gripper_delta",
        )

    def test_pose_update_is_atomic_keepalive_and_stop(self):
        bridge = Bridge()
        owner = object()
        request = {
            "owner": owner,
            "message": {"type": "pose_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
            "p_rel_m": [0.0, 0.0, 0.0],
            "q_rel": [0.0, 0.0, 0.0, 1.0],
            "gripper_delta": 0.0,
        }
        bridge.pending = request

        before = time.monotonic()
        p_rel_m = [0.01, 0.02, -0.03]
        q_rel = [0.0, 0.2588190451, 0.0, 0.9659258263]
        self.assertTrue(bridge.refresh_pose(owner, p_rel_m, q_rel, -0.4))
        self.assertGreater(request["deadline"], before)
        self.assertEqual(bridge.pose_snapshot(request), {
            "p_rel_m": p_rel_m,
            "q_rel": q_rel,
            "gripper_delta": -0.4,
        })
        self.assertEqual(
            bridge.request_pose_stop(owner),
            ("ok", "pose_stop_requested"),
        )
        self.assertTrue(bridge.pose_stop_event.is_set())

    def test_other_owner_cannot_update_or_stop_pose(self):
        bridge = Bridge()
        request = {
            "owner": object(),
            "message": {"type": "pose_start"},
            "future": concurrent.futures.Future(),
            "deadline": 0.0,
            "p_rel_m": [0.0, 0.0, 0.0],
            "q_rel": [0.0, 0.0, 0.0, 1.0],
            "gripper_delta": 0.0,
        }
        bridge.pending = request
        other_owner = object()
        self.assertFalse(
            bridge.refresh_pose(
                other_owner,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                0.0,
            )
        )
        self.assertEqual(
            bridge.request_pose_stop(other_owner),
            ("rejected", "pose_not_active"),
        )


if __name__ == "__main__":
    unittest.main()
