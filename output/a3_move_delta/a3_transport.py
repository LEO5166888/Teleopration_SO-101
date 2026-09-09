"""A-3 v0 WebSocket transport for SO-101 Cartesian translation.

This module owns validation, sequencing, state broadcasts, and JSONL logging.
It intentionally has no Isaac imports; the simulation thread is the only code
that reads robot state or executes a command.
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


PROTOCOL_VERSION = 0
SUPPORTED_TYPES = {
    "hello",
    "test",
    "move_delta",
    "gripper",
    "mode",
    "estop",
    "estop_reset",
}
TRANSLATION_AXES = {"x", "y", "z"}
ALL_AXES = TRANSLATION_AXES | {"rx", "ry", "rz"}
STEP_METERS = {"coarse": 0.020, "fine": 0.005}


def envelope(kind, seq, payload):
    return {
        "v": PROTOCOL_VERSION,
        "type": kind,
        "ts": int(time.time() * 1000),
        "seq": seq,
        "payload": payload,
    }


def validate_envelope(message):
    if not isinstance(message, dict):
        return -1, "invalid_message", "message must be a JSON object"

    seq = message.get("seq", -1)
    safe_seq = seq if type(seq) is int else -1
    required = {"v", "type", "ts", "seq", "payload"}
    missing = sorted(required - message.keys())
    if missing:
        return safe_seq, "missing_fields", f"missing fields: {missing}"
    if type(message["v"]) is not int or message["v"] != PROTOCOL_VERSION:
        return safe_seq, "unsupported_version", "only v=0 is supported"
    if not isinstance(message["type"], str):
        return safe_seq, "invalid_type", "type must be a string"
    if type(message["ts"]) is not int:
        return safe_seq, "invalid_ts", "ts must be an integer in milliseconds"
    if type(message["seq"]) is not int or message["seq"] < 0:
        return -1, "invalid_seq", "seq must be a non-negative integer"
    if not isinstance(message["payload"], dict):
        return safe_seq, "invalid_payload", "payload must be a JSON object"
    if message["type"] not in SUPPORTED_TYPES:
        return safe_seq, "unknown_type", f"unsupported type={message['type']}"
    return None


def validate_move_delta(payload):
    required = {"axis", "dir", "step"}
    if set(payload) != required:
        return "invalid_move_delta", "payload must contain only axis, dir, and step"
    axis = payload["axis"]
    direction = payload["dir"]
    step = payload["step"]
    if axis not in ALL_AXES:
        return "invalid_axis", "axis must be x/y/z/rx/ry/rz"
    if axis not in TRANSLATION_AXES:
        return "rotation_not_supported", "v0 executes x/y/z translation only"
    if type(direction) is not int or direction not in (-1, 1):
        return "invalid_dir", "dir must be integer -1 or 1"
    if step not in STEP_METERS:
        return "invalid_step", "step must be coarse or fine"
    return None


def validate_gripper(payload):
    if set(payload) != {"action"}:
        return "invalid_gripper", "payload must contain only action"
    if payload["action"] not in {"open", "close"}:
        return "invalid_gripper_action", "action must be open or close"
    return None


def validate_mode(payload):
    if set(payload) != {"step"} or payload["step"] not in STEP_METERS:
        return "invalid_mode", "payload.step must be coarse or fine"
    return None


def validate_empty(payload, kind):
    if payload != {}:
        return f"invalid_{kind}", "payload must be empty"
    return None


class Bridge:
    def __init__(self, host="127.0.0.1", port=8766, log_file=None):
        self.host = host
        self.port = port
        self.session = str(uuid.uuid4())
        self.commands = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.started = threading.Event()
        self.ready = threading.Event()
        self.start_error = None
        self.thread = None
        self.log_file = Path(log_file) if log_file else None
        self.log_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.pending = None
        self.estop_event = threading.Event()
        self.estop_frozen = threading.Event()
        self.state_lock = threading.Lock()
        self.state = {
            "ee_pos": None,
            "gripper": "closed",
            "step": "coarse",
            "ik": "initializing",
            "estop": False,
        }

    def log(self, direction, message):
        if self.log_file is None:
            return
        record = {
            "server_ts": int(time.time() * 1000),
            "direction": direction,
            "session": self.session,
            "message": message,
        }
        with self.log_lock:
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            with self.log_file.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def update_state(self, **fields):
        with self.state_lock:
            self.state.update(fields)

    def state_snapshot(self):
        with self.state_lock:
            return dict(self.state)

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
        async with serve(
            self.handler,
            self.host,
            self.port,
            max_size=8192,
            max_queue=4,
            close_timeout=2,
        ) as server:
            self.port = server.sockets[0].getsockname()[1]
            self.started.set()
            while not self.stop_event.is_set():
                await asyncio.sleep(0.05)

    async def send(self, ws, kind, seq, payload, send_lock=None):
        message = envelope(kind, seq, payload)
        self.log("out", message)
        text = json.dumps(message, ensure_ascii=False, allow_nan=False)
        if send_lock is None:
            await ws.send(text)
        else:
            async with send_lock:
                await ws.send(text)

    async def send_state(self, ws, seq, send_lock=None):
        state = self.state_snapshot()
        if state["ee_pos"] is not None:
            await self.send(ws, "state", seq, state, send_lock)

    async def complete_request(self, ws, send_lock, seq, future):
        try:
            result = await asyncio.wait_for(
                asyncio.shield(asyncio.wrap_future(future)), timeout=50.0
            )
        except asyncio.TimeoutError:
            result = {"status": "rejected", "reason": "sim_result_timeout"}
        try:
            await self.send(ws, "ack", seq, {
                "seq": seq,
                "status": result["status"],
                "reason": result["reason"],
            }, send_lock)
            await self.send_state(ws, seq, send_lock)
        except ConnectionClosed:
            pass

    async def handler(self, ws):
        welcomed = False
        highest_seq = -1
        send_lock = asyncio.Lock()
        completion_tasks = set()
        try:
            while True:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.2)
                except asyncio.TimeoutError:
                    if welcomed:
                        await self.send_state(ws, highest_seq, send_lock)
                    continue

                try:
                    message = json.loads(raw)
                except (ValueError, TypeError):
                    await self.send(ws, "error", -1, {
                        "code": "invalid_json",
                        "msg": "unable to parse JSON",
                    }, send_lock)
                    continue

                error = validate_envelope(message)
                if error is not None:
                    seq, code, msg = error
                    await self.send(
                        ws, "error", seq, {"code": code, "msg": msg}, send_lock
                    )
                    continue

                seq = message["seq"]
                if seq <= highest_seq:
                    self.log("drop_stale", message)
                    continue
                highest_seq = seq
                self.log("in", message)
                kind = message["type"]
                payload = message["payload"]

                if kind == "hello":
                    if set(payload) != {"client", "name"}:
                        await self.send(ws, "error", seq, {
                            "code": "invalid_hello",
                            "msg": "payload must contain client and name",
                        }, send_lock)
                        continue
                    if payload["client"] != "phone" or not isinstance(payload["name"], str):
                        await self.send(ws, "error", seq, {
                            "code": "invalid_hello",
                            "msg": "client must be phone and name must be a string",
                        }, send_lock)
                        continue
                    welcomed = True
                    await self.send(ws, "welcome", seq, {
                        "session": self.session,
                        "config": {"coarse_cm": 2.0, "fine_cm": 0.5},
                    }, send_lock)
                    await self.send_state(ws, seq, send_lock)
                    continue

                if not welcomed:
                    await self.send(ws, "error", seq, {
                        "code": "hello_required",
                        "msg": "send hello before commands",
                    }, send_lock)
                    continue

                if kind == "test":
                    async with send_lock:
                        await ws.send(raw)
                    self.log("out", message)
                    continue

                if kind in {"estop", "estop_reset"}:
                    command_error = validate_empty(payload, kind)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "rejected",
                            "reason": code,
                        }, send_lock)
                        continue
                    if kind == "estop":
                        self.engage_estop()
                        status = "ok"
                        reason = "estop_engaged"
                    else:
                        status, reason = self.reset_estop()
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": status,
                        "reason": reason,
                    }, send_lock)
                    await self.send_state(ws, seq, send_lock)
                    continue

                if kind == "mode":
                    command_error = validate_mode(payload)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "rejected",
                            "reason": code,
                        }, send_lock)
                    else:
                        self.update_state(step=payload["step"])
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "ok",
                            "reason": "mode_updated",
                        }, send_lock)
                        await self.send_state(ws, seq, send_lock)
                    continue

                if self.estop_event.is_set():
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": "rejected",
                        "reason": "estop_active",
                    }, send_lock)
                    continue

                if kind == "move_delta":
                    command_error = validate_move_delta(payload)
                else:
                    command_error = validate_gripper(payload)
                if command_error is not None:
                    code, reason = command_error
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": "rejected",
                        "reason": code,
                    }, send_lock)
                    continue
                if not self.ready.is_set():
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": "rejected",
                        "reason": "sim_not_ready",
                    }, send_lock)
                    continue

                future = concurrent.futures.Future()
                with self.command_lock:
                    if (
                        self.pending is not None
                        and not self.pending["future"].done()
                    ):
                        busy = True
                    else:
                        busy = False
                        request = {
                            "seq": seq,
                            "message": message,
                            "future": future,
                            "deadline": time.monotonic() + 45.0,
                            "owner": ws,
                        }
                        self.pending = request
                        self.commands.put_nowait(request)
                if busy:
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": "rejected",
                        "reason": "busy",
                    }, send_lock)
                    continue

                task = asyncio.create_task(
                    self.complete_request(ws, send_lock, seq, future)
                )
                completion_tasks.add(task)
                task.add_done_callback(completion_tasks.discard)
        except ConnectionClosed:
            pass
        finally:
            # A dropped control connection must interrupt only the command
            # owned by that connection. An idle disconnect remains harmless.
            with self.command_lock:
                disconnected_request = self.pending
                must_stop = (
                    disconnected_request is not None
                    and disconnected_request.get("owner") is ws
                    and not disconnected_request["future"].done()
                )
            if must_stop:
                self.log("disconnect_estop", {
                    "seq": disconnected_request["seq"],
                    "reason": "control_connection_lost",
                })
                self.engage_estop()

    def take(self):
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return None

    def engage_estop(self):
        self.estop_frozen.clear()
        self.estop_event.set()
        self.update_state(estop=True, ik="stopped")
        try:
            queued = self.commands.get_nowait()
        except queue.Empty:
            return
        self.finish(queued, "rejected", "estop")

    def mark_estop_frozen(self):
        if self.estop_event.is_set():
            self.estop_frozen.set()

    def reset_estop(self):
        if not self.estop_event.is_set():
            return "rejected", "estop_not_active"
        if not self.estop_frozen.is_set():
            return "rejected", "estop_not_frozen"
        self.estop_event.clear()
        self.estop_frozen.clear()
        self.update_state(estop=False, ik="ok")
        return "ok", "estop_reset"

    def finish(self, request, status, reason, diagnostics=None):
        result = {"status": status, "reason": reason}
        self.log("sim_result", {
            "seq": request["seq"],
            **result,
            "diagnostics": diagnostics or {},
        })
        future = request["future"]
        if not future.done():
            future.set_result(result)
        with self.command_lock:
            if self.pending is request:
                self.pending = None

    def close(self):
        self.ready.clear()
        self.update_state(ik="stopped")
        with self.command_lock:
            pending = self.pending
        if pending is not None and not pending["future"].done():
            pending["future"].set_result({
                "status": "rejected",
                "reason": "sim_stopped",
            })
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
