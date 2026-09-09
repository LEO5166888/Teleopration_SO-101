"""Temporary A-3 smoke protocol; no Isaac imports and no robot calls here.

Exactly one test_move_up is accepted per server process. It requests the fixed
base-frame delta [0, 0, 0.02] m. This isn't Thomas's move_delta contract.
"""
import asyncio
import concurrent.futures
import json
import queue
import threading
import time
import uuid
from pathlib import Path

from websockets.exceptions import ConnectionClosed
from websockets.legacy.server import serve


def envelope(kind, seq, payload):
    return dict(v=0, type=kind, ts=int(time.time() * 1000), seq=seq, payload=payload)


class Bridge:
    def __init__(self, host="127.0.0.1", port=8766, log_file=None):
        self.host, self.port = host, port
        self.session = str(uuid.uuid4())
        self.commands = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.started = threading.Event()
        self.ready = threading.Event()
        self.start_error = None
        self.used = False  # Accessed only in the network event loop.
        self.pending = None
        self.log_file = Path(log_file) if log_file else None
        self.log_lock = threading.Lock()
        self.thread = None

    def log(self, direction, message):
        if self.log_file:
            record = dict(server_ts=int(time.time() * 1000), direction=direction,
                          session=self.session, message=message)
            with self.log_lock:
                self.log_file.parent.mkdir(parents=True, exist_ok=True)
                with self.log_file.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        if not self.started.wait(10):
            raise RuntimeError("WebSocket startup timed out")
        if self.start_error:
            raise RuntimeError(f"WebSocket startup failed: {self.start_error}")

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self.start_error = exc
            self.started.set()

    async def _serve(self):
        async with serve(self.handler, self.host, self.port, max_size=8192,
                         max_queue=4, close_timeout=2) as server:
            self.port = server.sockets[0].getsockname()[1]
            self.started.set()
            while not self.stop_event.is_set():
                await asyncio.sleep(0.05)

    async def send(self, ws, kind, seq, payload):
        message = envelope(kind, seq, payload)
        self.log("out", message)
        await ws.send(json.dumps(message, allow_nan=False))

    async def handler(self, ws):
        welcomed = False
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except (ValueError, TypeError):
                    await self.send(ws, "error", -1, {"code": "invalid_json"})
                    continue
                # Check types before using .get or logging non-finite JSON.
                if not isinstance(message, dict):
                    await self.send(ws, "error", -1, {"code": "invalid_message"})
                    continue
                seq = message.get("seq", -1)
                if type(seq) is not int:
                    seq = -1
                if (type(message.get("v")) is not int or message["v"] != 0
                        or type(message.get("seq")) is not int or seq < 0
                        or type(message.get("ts")) is not int
                        or not isinstance(message.get("type"), str)
                        or not isinstance(message.get("payload"), dict)):
                    await self.send(ws, "error", seq, {"code": "invalid_envelope"})
                    continue
                kind = message["type"]
                if kind not in ("hello", "test_move_up"):
                    await self.send(ws, "error", seq, {"code": "unknown_type"})
                    continue
                if message["payload"] != {}:
                    await self.send(ws, "error", seq, {"code": "payload_must_be_empty"})
                    continue
                self.log("in", message)
                if kind == "hello":
                    welcomed = True
                    await self.send(ws, "welcome", seq, {
                        "session": self.session, "protocol": "a3-smoke-1",
                        "ready": self.ready.is_set(), "used": self.used,
                        "frame": "base", "delta_m": [0.0, 0.0, 0.02],
                    })
                elif not welcomed or not self.ready.is_set():
                    await self.send(ws, "error", seq, {"code": "not_ready"})
                elif self.used:
                    await self.send(ws, "error", seq, {"code": "one_shot_used_restart_server"})
                else:
                    future = concurrent.futures.Future()
                    request = {"seq": seq, "future": future,
                               "deadline": time.monotonic() + 45.0}
                    self.used = True
                    self.pending = future
                    self.commands.put_nowait(request)
                    await self.send(ws, "ack", seq, {"status": "accepted"})
                    # ACK only means admitted. Only the sim thread produces result.
                    try:
                        result = await asyncio.wait_for(
                            asyncio.shield(asyncio.wrap_future(future)), timeout=50)
                    except asyncio.TimeoutError:
                        await self.send(ws, "error", seq, {"code": "sim_result_timeout"})
                    else:
                        await self.send(ws, "result", seq, result)
        except ConnectionClosed:
            pass

    def take(self):
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return None

    def finish(self, request, result):
        self.log("sim_result", dict(seq=request["seq"], **result))
        if not request["future"].done():
            request["future"].set_result(result)

    def close(self):
        self.ready.clear()
        if self.pending is not None and not self.pending.done():
            self.pending.set_result({"ok": False, "reason": "sim_stopped"})
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
