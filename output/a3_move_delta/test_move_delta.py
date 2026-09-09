"""Protocol and Mac-client tests with a fake simulation worker."""

import asyncio
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from websockets.legacy.client import connect

from a3_transport import (
    Bridge,
    envelope,
    validate_envelope,
    validate_gripper,
    validate_move_delta,
)
from mac_disconnect_estop import run as run_disconnect_estop
from mac_estop import run as run_estop
from mac_gripper import run as run_gripper
from mac_move_delta import run, verify_motion


HELLO = {"client": "phone", "name": "unit-test"}


class ValidationTest(unittest.TestCase):
    def test_envelope_and_move_delta_validation(self):
        self.assertIsNone(validate_envelope(envelope("hello", 1, HELLO)))
        self.assertIsNotNone(validate_envelope([]))
        self.assertIsNotNone(validate_envelope({"v": 0}))
        self.assertIsNone(validate_move_delta({
            "axis": "x", "dir": 1, "step": "coarse"
        }))
        for payload in (
            {"axis": "rx", "dir": 1, "step": "coarse"},
            {"axis": "z", "dir": True, "step": "coarse"},
            {"axis": "z", "dir": 1, "step": "large"},
            {"axis": "z", "dir": 1, "step": "fine", "extra": 1},
        ):
            self.assertIsNotNone(validate_move_delta(payload))
        self.assertIsNone(validate_gripper({"action": "open"}))
        self.assertIsNone(validate_gripper({"action": "close"}))
        self.assertIsNotNone(validate_gripper({"action": "half"}))

    def test_motion_verifier(self):
        actual, expected, error = verify_motion(
            [0.1, -0.2, 0.3],
            [0.1, -0.2, 0.319],
            "z",
            1,
            "coarse",
            "ok",
        )
        self.assertEqual(expected, [0.0, 0.0, 0.02])
        self.assertAlmostEqual(actual[2], 0.019)
        self.assertLess(error, 0.005)
        with self.assertRaises(ValueError):
            verify_motion(
                [0.1, -0.2, 0.3],
                [0.12, -0.2, 0.3],
                "z",
                1,
                "coarse",
                "ok",
            )
        with self.assertRaises(ValueError):
            verify_motion(
                [0.1, -0.2, 0.3],
                [0.1, -0.2, 0.3],
                "z",
                1,
                "fine",
                "ok",
            )


class BridgeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bridge = Bridge(port=0, log_file=Path(self.temp.name) / "test.jsonl")
        self.bridge.update_state(ee_pos=[0.1, -0.2, 0.3], ik="ok")
        self.bridge.ready.set()
        self.bridge.start()
        self.uri = f"ws://127.0.0.1:{self.bridge.port}"

    async def asyncTearDown(self):
        await asyncio.to_thread(self.bridge.close)
        self.temp.cleanup()

    async def receive(self, ws, timeout=3):
        return json.loads(await asyncio.wait_for(ws.recv(), timeout))

    async def receive_ack(self, ws, seq, timeout=3):
        deadline = asyncio.get_running_loop().time() + timeout
        while True:
            reply = await self.receive(
                ws,
                max(0.01, deadline - asyncio.get_running_loop().time()),
            )
            if reply.get("type") == "ack" and reply.get("seq") == seq:
                return reply

    async def hello(self, ws):
        await ws.send(json.dumps(envelope("hello", 1, HELLO)))
        welcome = await self.receive(ws)
        state = await self.receive(ws)
        self.assertEqual(welcome["type"], "welcome")
        self.assertEqual(
            welcome["payload"]["config"],
            {"coarse_cm": 2.0, "fine_cm": 0.5},
        )
        self.assertEqual(state["type"], "state")

    async def test_reject_rotation_without_queueing(self):
        async with connect(self.uri) as ws:
            await self.hello(ws)
            await ws.send(json.dumps(envelope("move_delta", 2, {
                "axis": "rx", "dir": 1, "step": "coarse"
            })))
            reply = await self.receive(ws)
            self.assertEqual(reply["type"], "ack")
            self.assertEqual(reply["payload"]["status"], "rejected")
            self.assertEqual(reply["payload"]["reason"], "rotation_not_supported")
            self.assertIsNone(self.bridge.take())

    async def test_reset_is_rejected_until_sim_confirms_freeze(self):
        async with connect(self.uri) as ws:
            await self.hello(ws)
            await ws.send(json.dumps(envelope("estop", 2, {})))
            engaged = await self.receive_ack(ws, 2)
            self.assertEqual(engaged["payload"]["status"], "ok")

            await ws.send(json.dumps(envelope("estop_reset", 3, {})))
            early_reset = await self.receive_ack(ws, 3)
            self.assertEqual(early_reset["payload"]["status"], "rejected")
            self.assertEqual(early_reset["payload"]["reason"], "estop_not_frozen")

            self.bridge.mark_estop_frozen()
            await ws.send(json.dumps(envelope("estop_reset", 4, {})))
            reset = await self.receive_ack(ws, 4)
            self.assertEqual(reset["payload"]["status"], "ok")
            self.assertEqual(reset["payload"]["reason"], "estop_reset")

    async def test_ack_is_final_and_client_checks_state_motion(self):
        async def fake_sim():
            for _ in range(300):
                request = self.bridge.take()
                if request is not None:
                    self.assertEqual(request["message"]["payload"], {
                        "axis": "z", "dir": 1, "step": "coarse"
                    })
                    self.bridge.update_state(ee_pos=[0.1, -0.2, 0.319], ik="ok")
                    self.bridge.finish(
                        request,
                        "ok",
                        "completed",
                        {"position_error_m": 0.001},
                    )
                    return
                await asyncio.sleep(0.01)
            raise AssertionError("client command never reached fake simulation")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            await asyncio.gather(
                run(self.uri, "z", 1, "coarse"),
                fake_sim(),
            )
        self.assertIn("A3 MOVE_DELTA Z PLUS COARSE PASS", output.getvalue())
        self.assertFalse(self.bridge.estop_event.is_set())
        records = [
            json.loads(line)
            for line in self.bridge.log_file.read_text(encoding="utf-8").splitlines()
        ]
        self.assertTrue(any(record["direction"] == "sim_result" for record in records))

    async def test_gripper_client_checks_final_state(self):
        async def fake_sim():
            for _ in range(300):
                request = self.bridge.take()
                if request is not None:
                    self.assertEqual(request["message"]["payload"], {"action": "open"})
                    self.bridge.update_state(gripper="open")
                    self.bridge.finish(
                        request,
                        "ok",
                        "completed",
                        {"start_deg": 0.0, "target_deg": 100.0, "end_deg": 99.9},
                    )
                    return
                await asyncio.sleep(0.01)
            raise AssertionError("gripper command never reached fake simulation")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            await asyncio.gather(run_gripper(self.uri, "open"), fake_sim())
        self.assertIn("A3 GRIPPER OPEN PASS", output.getvalue())

    async def test_estop_interrupts_same_connection_and_requires_reset(self):
        async def fake_sim():
            for _ in range(300):
                request = self.bridge.take()
                if request is not None:
                    self.bridge.update_state(
                        ee_pos=[0.1, -0.2, 0.303],
                        ik="moving",
                    )
                    break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError("move command never reached fake simulation")

            for _ in range(300):
                if self.bridge.estop_event.is_set():
                    self.bridge.update_state(
                        ee_pos=[0.1, -0.2, 0.303],
                        estop=True,
                        ik="stopped",
                    )
                    self.bridge.mark_estop_frozen()
                    self.bridge.finish(request, "rejected", "estop")
                    return
                await asyncio.sleep(0.01)
            raise AssertionError("fake simulation did not receive estop")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            await asyncio.gather(run_estop(self.uri), fake_sim())
        self.assertIn(
            "A3 ESTOP SAME-CONNECTION INTERRUPT PASS",
            output.getvalue(),
        )

    async def test_disconnect_during_owned_command_engages_estop(self):
        async with connect(self.uri) as ws:
            await self.hello(ws)
            await ws.send(json.dumps(envelope("move_delta", 2, {
                "axis": "z", "dir": 1, "step": "coarse"
            })))
            for _ in range(100):
                request = self.bridge.take()
                if request is not None:
                    break
                await asyncio.sleep(0.01)
            else:
                self.fail("move command never reached fake simulation")

        for _ in range(100):
            if self.bridge.estop_event.is_set():
                break
            await asyncio.sleep(0.01)
        self.assertTrue(self.bridge.estop_event.is_set())
        self.bridge.mark_estop_frozen()
        self.bridge.finish(request, "rejected", "estop")

    async def test_disconnect_client_verifies_freeze_block_and_reset(self):
        async def fake_sim():
            for _ in range(300):
                request = self.bridge.take()
                if request is not None:
                    self.bridge.update_state(
                        ee_pos=[0.1, -0.2, 0.303],
                        ik="moving",
                    )
                    break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError("move command never reached fake simulation")

            for _ in range(300):
                if self.bridge.estop_event.is_set():
                    self.bridge.update_state(
                        ee_pos=[0.1, -0.2, 0.303],
                        estop=True,
                        ik="stopped",
                    )
                    self.bridge.mark_estop_frozen()
                    self.bridge.finish(request, "rejected", "estop")
                    return
                await asyncio.sleep(0.01)
            raise AssertionError("disconnect did not engage estop")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            await asyncio.gather(run_disconnect_estop(self.uri), fake_sim())
        self.assertIn("A3 DISCONNECT ESTOP PASS", output.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
