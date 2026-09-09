"""Local network/protocol checks with a fake sim worker. No physics validation."""
import asyncio
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from websockets.legacy.client import connect

from a3_transport import Bridge, envelope
from mac_test_up import run, verify_result


def result():
    return {"ok": True, "frame": "base", "start_m": [0.1, -0.2, 0.3],
            "target_m": [0.1, -0.2, 0.32], "end_m": [0.1, -0.2, 0.319]}


class TransportTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bridge = Bridge(port=0, log_file=Path(self.temp.name) / "test.jsonl")
        self.bridge.start()
        self.uri = f"ws://127.0.0.1:{self.bridge.port}"

    async def asyncTearDown(self):
        await asyncio.to_thread(self.bridge.close)
        self.temp.cleanup()

    async def receive(self, ws):
        return json.loads(await asyncio.wait_for(ws.recv(), 3))

    async def exchange(self, ws, message):
        await ws.send(json.dumps(message))
        return await self.receive(ws)

    async def test_validation_and_readiness(self):
        async with connect(self.uri) as ws:
            for value in ([], None, 3, "text"):
                reply = await self.exchange(ws, value)
                self.assertEqual(reply["payload"]["code"], "invalid_message")
            await ws.send("{")
            self.assertEqual((await self.receive(ws))["payload"]["code"], "invalid_json")
            bad = envelope("hello", True, {})
            self.assertEqual((await self.exchange(ws, bad))["type"], "error")
            reply = await self.exchange(ws, envelope("test_move_up", 2, {}))
            self.assertEqual(reply["payload"]["code"], "not_ready")
            self.bridge.ready.set()
            await self.exchange(ws, envelope("hello", 1, {}))
            for kind, payload in (("move_delta", {}), ("test_move_up", {"z": 10})):
                self.assertEqual((await self.exchange(ws, envelope(kind, 3, payload)))["type"], "error")
            self.assertIsNone(self.bridge.take())

    async def test_ack_does_not_mean_execution_and_second_move_rejected(self):
        self.bridge.ready.set()
        async with connect(self.uri) as ws:
            await self.exchange(ws, envelope("hello", 1, {}))
            reply = await self.exchange(ws, envelope("test_move_up", 2, {}))
            self.assertEqual(reply["type"], "ack")
            request = self.bridge.take()
            self.assertIsNotNone(request)
            with self.assertRaises(asyncio.TimeoutError):
                await asyncio.wait_for(ws.recv(), 0.1)
            self.bridge.finish(request, {"ok": False, "reason": "fake_ik_failure"})
            reply = await self.receive(ws)
            self.assertFalse(reply["payload"]["ok"])
            retry = await self.exchange(ws, envelope("test_move_up", 3, {}))
            self.assertEqual(retry["payload"]["code"], "one_shot_used_restart_server")
            self.assertIsNone(self.bridge.take())

    async def test_disconnect_does_not_allow_duplicate_move(self):
        self.bridge.ready.set()
        async with connect(self.uri) as first:
            await self.exchange(first, envelope("hello", 1, {}))
            await self.exchange(first, envelope("test_move_up", 2, {}))
        request = self.bridge.take()
        async with connect(self.uri) as second:
            welcome = await self.exchange(second, envelope("hello", 1, {}))
            self.assertTrue(welcome["payload"]["used"])
            retry = await self.exchange(second, envelope("test_move_up", 2, {}))
            self.assertEqual(retry["type"], "error")
        self.bridge.finish(request, result())
        self.assertIsNone(self.bridge.take())

    async def test_real_client_fake_sim_roundtrip(self):
        self.bridge.ready.set()

        async def fake_sim():
            for _ in range(300):
                request = self.bridge.take()
                if request is not None:
                    self.bridge.finish(request, result())
                    return
                await asyncio.sleep(0.01)
            raise AssertionError("No request arrived")

        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            await asyncio.gather(run(self.uri), fake_sim())
        self.assertIn("A3 MAC -> SSH -> SO101 UP 2CM PASS", buffer.getvalue())
        records = [json.loads(line) for line in self.bridge.log_file.read_text().splitlines()]
        self.assertTrue(any(r["direction"] == "sim_result" for r in records))


class ResultTest(unittest.TestCase):
    def test_zero_motion_wrong_axis_nan_and_missing_measurement_fail(self):
        for end in ([0.1, -0.2, 0.3], [0.12, -0.2, 0.3], [float("nan"), 0, 0], None):
            payload = result()
            payload["end_m"] = end
            with self.assertRaises(ValueError):
                verify_result(payload)
        with self.assertRaises(ValueError):
            verify_result({"ok": True, "frame": "base"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
