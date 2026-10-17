"""A-3 WebSocket transport for SO-101 Cartesian translation.

This module owns validation, sequencing, state broadcasts, and JSONL logging.
It intentionally has no Isaac imports; the simulation thread is the only code
that reads robot state or executes a command.  Protocol v0 remains supported
for the existing Mac tools; Thomas's App uses the schema-compatible v1.
"""

import asyncio
import concurrent.futures
import json
import math
import queue
import threading
import time
import uuid
from pathlib import Path

from websockets.exceptions import ConnectionClosed
from websockets.legacy.server import serve


PROTOCOL_VERSION = 0
SUPPORTED_PROTOCOL_VERSIONS = {0, 1}
SUPPORTED_TYPES = {
    "hello",
    "test",
    "move_delta",
    "jog_start",
    "jog_keepalive",
    "jog_stop",
    "joint_jog_start",
    "joint_jog_stop",
    "gripper_jog_start",
    "gripper_jog_stop",
    "orient_start",
    "orient_update",
    "orient_stop",
    "pose_start",
    "pose_update",
    "pose_stop",
    "joint_delta",
    "gripper",
    "mode",
    "estop",
    "estop_reset",
    "reset_scene",
}
JOG_WATCHDOG_S = 0.75
TRANSLATION_AXES = {"x", "y", "z"}
ALL_AXES = TRANSLATION_AXES | {"rx", "ry", "rz"}
STEP_METERS = {"coarse": 0.020, "fine": 0.005}
ARM_JOINT_NAMES = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
}


def envelope(kind, seq, payload, version=PROTOCOL_VERSION):
    return {
        "v": version,
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
    if (
        type(message["v"]) is not int
        or message["v"] not in SUPPORTED_PROTOCOL_VERSIONS
    ):
        return safe_seq, "unsupported_version", "supported versions are v=0 and v=1"
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
    allowed = required | {"frame"}
    if not required.issubset(payload) or not set(payload).issubset(allowed):
        return (
            "invalid_move_delta",
            "payload must contain axis, dir, and step; frame is optional",
        )
    axis = payload["axis"]
    direction = payload["dir"]
    step = payload["step"]
    frame = payload.get("frame", "base")
    if axis not in ALL_AXES:
        return "invalid_axis", "axis must be x/y/z/rx/ry/rz"
    if axis not in TRANSLATION_AXES:
        return "rotation_not_supported", "v0 executes x/y/z translation only"
    if type(direction) is not int or direction not in (-1, 1):
        return "invalid_dir", "dir must be integer -1 or 1"
    if step not in STEP_METERS:
        return "invalid_step", "step must be coarse or fine"
    if frame not in {"base", "tool"}:
        return "invalid_frame", "frame must be base or tool"
    return None


def validate_jog_start(payload):
    error = validate_move_delta(payload)
    if error is None:
        return None
    code, reason = error
    return f"jog_{code}", reason


def validate_gripper(payload):
    if set(payload) != {"action"}:
        return "invalid_gripper", "payload must contain only action"
    if payload["action"] not in {"open", "close"}:
        return "invalid_gripper_action", "action must be open or close"
    return None


def validate_gripper_jog_start(payload):
    if set(payload) != {"action", "step"}:
        return (
            "invalid_gripper_jog_start",
            "payload must contain only action and step",
        )
    if payload["action"] not in {"open", "close"}:
        return "invalid_gripper_action", "action must be open or close"
    if payload["step"] not in STEP_METERS:
        return "invalid_step", "step must be coarse or fine"
    return None


def validate_joint_delta(payload):
    if set(payload) != {"joint", "dir", "step"}:
        return (
            "invalid_joint_delta",
            "payload must contain only joint, dir, and step",
        )
    if payload["joint"] not in ARM_JOINT_NAMES:
        return "invalid_joint", "joint is not an adjustable arm joint"
    if type(payload["dir"]) is not int or payload["dir"] not in (-1, 1):
        return "invalid_dir", "dir must be integer -1 or 1"
    if payload["step"] not in STEP_METERS:
        return "invalid_step", "step must be coarse or fine"
    return None


def validate_mode(payload):
    if set(payload) != {"step"} or payload["step"] not in STEP_METERS:
        return "invalid_mode", "payload.step must be coarse or fine"
    return None


def validate_empty(payload, kind):
    if payload != {}:
        return f"invalid_{kind}", "payload must be empty"
    return None


def _finite_number(value):
    return type(value) in {int, float} and math.isfinite(value)


def validate_orient_start(payload):
    required = {"frame", "rate_hz", "gain", "deadband_deg", "smoothing"}
    if set(payload) != required:
        return (
            "invalid_orient_start",
            "payload must contain frame, rate_hz, gain, deadband_deg, and smoothing",
        )
    if payload["frame"] != "phone_body":
        return "invalid_orient_frame", "frame must be phone_body"
    if type(payload["rate_hz"]) is not int or payload["rate_hz"] != 30:
        return "invalid_orient_rate", "rate_hz must be integer 30"
    limits = {
        "gain": (0.1, 2.0),
        "deadband_deg": (0.0, 15.0),
        "smoothing": (0.0, 0.95),
    }
    for field, (lower, upper) in limits.items():
        value = payload[field]
        if not _finite_number(value) or not lower <= value <= upper:
            return (
                "invalid_orient_parameter",
                f"{field} must be finite and within [{lower}, {upper}]",
            )
    return None


def validate_orient_update(payload):
    if set(payload) != {"q_rel"}:
        return "invalid_orient_update", "payload must contain only q_rel"
    q_rel = payload["q_rel"]
    if (
        not isinstance(q_rel, list)
        or len(q_rel) != 4
        or not all(_finite_number(value) for value in q_rel)
    ):
        return "invalid_orient_quaternion", "q_rel must be four finite numbers"
    norm = math.sqrt(sum(value * value for value in q_rel))
    if not 0.98 <= norm <= 1.02:
        return "invalid_orient_quaternion", "q_rel must be a unit quaternion"
    return None


def validate_pose_start(payload):
    required = {"frame", "rate_hz", "gain", "deadband_deg", "smoothing"}
    if set(payload) != required:
        return (
            "invalid_pose_start",
            "payload must contain frame, rate_hz, gain, deadband_deg, and smoothing",
        )
    if payload["frame"] != "webxr_local":
        return "invalid_pose_frame", "frame must be webxr_local"
    if type(payload["rate_hz"]) is not int or payload["rate_hz"] != 30:
        return "invalid_pose_rate", "rate_hz must be integer 30"
    limits = {
        "gain": (0.1, 2.0),
        "deadband_deg": (0.0, 15.0),
        "smoothing": (0.0, 0.95),
    }
    for field, (lower, upper) in limits.items():
        value = payload[field]
        if not _finite_number(value) or not lower <= value <= upper:
            return (
                "invalid_pose_parameter",
                f"{field} must be finite and within [{lower}, {upper}]",
            )
    return None


def validate_pose_update(payload):
    if set(payload) != {"p_rel_m", "q_rel", "gripper_delta"}:
        return (
            "invalid_pose_update",
            "payload must contain only p_rel_m, q_rel, and gripper_delta",
        )
    p_rel_m = payload["p_rel_m"]
    if (
        not isinstance(p_rel_m, list)
        or len(p_rel_m) != 3
        or not all(_finite_number(value) for value in p_rel_m)
    ):
        return "invalid_pose_position", "p_rel_m must be three finite numbers"
    quaternion_error = validate_orient_update({"q_rel": payload["q_rel"]})
    if quaternion_error is not None:
        return "invalid_pose_quaternion", quaternion_error[1]
    gripper_delta = payload["gripper_delta"]
    if not _finite_number(gripper_delta) or not -1.0 <= gripper_delta <= 1.0:
        return (
            "invalid_pose_gripper_delta",
            "gripper_delta must be finite and within [-1, 1]",
        )
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
        self.jog_stop_event = threading.Event()
        self.joint_jog_stop_event = threading.Event()
        self.gripper_jog_stop_event = threading.Event()
        self.orient_stop_event = threading.Event()
        self.pose_stop_event = threading.Event()
        # Each client receives responses in the same protocol version it used.
        # This keeps the v0 Mac tools and Thomas's v1 App interoperable.
        self.connection_versions = {}
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
        message = envelope(
            kind,
            seq,
            payload,
            version=self.connection_versions.get(ws, PROTOCOL_VERSION),
        )
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

    async def complete_request(self, ws, send_lock, seq, future, timeout_s=50.0):
        try:
            wrapped = asyncio.shield(asyncio.wrap_future(future))
            if timeout_s is None:
                result = await wrapped
            else:
                result = await asyncio.wait_for(wrapped, timeout=timeout_s)
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

                negotiated_version = self.connection_versions.get(ws)
                if negotiated_version is None:
                    self.connection_versions[ws] = message["v"]
                elif message["v"] != negotiated_version:
                    await self.send(ws, "error", message["seq"], {
                        "code": "version_changed",
                        "msg": "protocol version must remain constant per connection",
                    }, send_lock)
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

                if kind in {
                    "jog_keepalive",
                    "jog_stop",
                    "joint_jog_stop",
                    "gripper_jog_stop",
                    "orient_stop",
                    "pose_stop",
                }:
                    command_error = validate_empty(payload, kind)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "rejected",
                            "reason": code,
                        }, send_lock)
                        continue
                    if self.estop_event.is_set():
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "rejected",
                            "reason": "estop_active",
                        }, send_lock)
                        continue
                    if kind == "jog_keepalive":
                        self.refresh_hold(ws)
                        continue
                    if kind == "jog_stop":
                        status, reason = self.request_jog_stop(ws)
                    elif kind == "joint_jog_stop":
                        status, reason = self.request_joint_jog_stop(ws)
                    elif kind == "gripper_jog_stop":
                        status, reason = self.request_gripper_jog_stop(ws)
                    elif kind == "orient_stop":
                        status, reason = self.request_orient_stop(ws)
                    else:
                        status, reason = self.request_pose_stop(ws)
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": status,
                        "reason": reason,
                    }, send_lock)
                    await self.send_state(ws, seq, send_lock)
                    continue


                if kind == "orient_update":
                    command_error = validate_orient_update(payload)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "error", seq, {
                            "code": code,
                            "msg": reason,
                        }, send_lock)
                        continue
                    # orient_update is both the newest target and the keepalive.
                    # It deliberately has no ACK at 30 Hz.
                    self.refresh_orient(ws, payload["q_rel"])
                    continue

                if kind == "pose_update":
                    command_error = validate_pose_update(payload)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "error", seq, {
                            "code": code,
                            "msg": reason,
                        }, send_lock)
                        continue
                    # pose_update is an atomic latest-wins target and keepalive.
                    # It deliberately has no ACK at 30 Hz.
                    self.refresh_pose(
                        ws,
                        payload["p_rel_m"],
                        payload["q_rel"],
                        payload["gripper_delta"],
                    )
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

                if kind == "reset_scene":
                    command_error = validate_empty(payload, kind)
                    if command_error is not None:
                        code, reason = command_error
                        await self.send(ws, "ack", seq, {
                            "seq": seq,
                            "status": "rejected",
                            "reason": code,
                        }, send_lock)
                        continue
                    self.prepare_scene_reset()

                if self.estop_event.is_set():
                    await self.send(ws, "ack", seq, {
                        "seq": seq,
                        "status": "rejected",
                        "reason": "estop_active",
                    }, send_lock)
                    continue

                if kind == "move_delta":
                    command_error = validate_move_delta(payload)
                elif kind == "jog_start":
                    command_error = validate_jog_start(payload)
                elif kind == "joint_jog_start":
                    command_error = validate_joint_delta(payload)
                elif kind == "gripper_jog_start":
                    command_error = validate_gripper_jog_start(payload)
                elif kind == "orient_start":
                    command_error = validate_orient_start(payload)
                elif kind == "pose_start":
                    command_error = validate_pose_start(payload)
                elif kind == "joint_delta":
                    command_error = validate_joint_delta(payload)
                elif kind == "gripper":
                    command_error = validate_gripper(payload)
                else:
                    command_error = None
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
                            "deadline": time.monotonic() + (
                                JOG_WATCHDOG_S
                                if kind in {
                                    "jog_start",
                                    "joint_jog_start",
                                    "gripper_jog_start",
                                    "orient_start",
                                    "pose_start",
                                }
                                else 45.0
                            ),
                            "owner": ws,
                        }
                        if kind == "jog_start":
                            self.jog_stop_event.clear()
                        elif kind == "joint_jog_start":
                            self.joint_jog_stop_event.clear()
                        elif kind == "gripper_jog_start":
                            self.gripper_jog_stop_event.clear()
                        elif kind == "orient_start":
                            self.orient_stop_event.clear()
                            request["q_rel"] = [0.0, 0.0, 0.0, 1.0]
                        elif kind == "pose_start":
                            self.pose_stop_event.clear()
                            request["p_rel_m"] = [0.0, 0.0, 0.0]
                            request["q_rel"] = [0.0, 0.0, 0.0, 1.0]
                            request["gripper_delta"] = 0.0
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
                    self.complete_request(
                        ws,
                        send_lock,
                        seq,
                        future,
                        timeout_s=(
                            None
                            if kind in {
                                "jog_start",
                                "joint_jog_start",
                                "gripper_jog_start",
                                "orient_start",
                                "pose_start",
                            }
                            else 50.0
                        ),
                    )
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
            self.connection_versions.pop(ws, None)

    def take(self):
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return None

    def engage_estop(self):
        self.estop_frozen.clear()
        self.estop_event.set()
        self.jog_stop_event.set()
        self.joint_jog_stop_event.set()
        self.gripper_jog_stop_event.set()
        self.orient_stop_event.set()
        self.pose_stop_event.set()
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

    def prepare_scene_reset(self):
        """Allow the simulation thread to perform a full reset from estop."""
        self.estop_event.clear()
        self.estop_frozen.clear()
        self.jog_stop_event.clear()
        self.joint_jog_stop_event.clear()
        self.gripper_jog_stop_event.clear()
        self.orient_stop_event.clear()
        self.pose_stop_event.clear()
        self.update_state(estop=False, ik="resetting")

    def refresh_hold(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"]
                not in {
                    "jog_start",
                    "joint_jog_start",
                    "gripper_jog_start",
                }
                or request["future"].done()
            ):
                return False
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return True

    def request_jog_stop(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "jog_start"
                or request["future"].done()
            ):
                return "rejected", "jog_not_active"
            self.jog_stop_event.set()
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return "ok", "jog_stop_requested"

    def request_gripper_jog_stop(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "gripper_jog_start"
                or request["future"].done()
            ):
                return "rejected", "gripper_jog_not_active"
            self.gripper_jog_stop_event.set()
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return "ok", "gripper_jog_stop_requested"

    def request_joint_jog_stop(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "joint_jog_start"
                or request["future"].done()
            ):
                return "rejected", "joint_jog_not_active"
            self.joint_jog_stop_event.set()
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return "ok", "joint_jog_stop_requested"

    def refresh_orient(self, owner, q_rel):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "orient_start"
                or request["future"].done()
            ):
                return False
            request["q_rel"] = list(q_rel)
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return True

    def orient_snapshot(self, request):
        with self.command_lock:
            if self.pending is not request or request["future"].done():
                return None
            return list(request.get("q_rel", [0.0, 0.0, 0.0, 1.0]))

    def request_orient_stop(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "orient_start"
                or request["future"].done()
            ):
                return "rejected", "orient_not_active"
            self.orient_stop_event.set()
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return "ok", "orient_stop_requested"

    def refresh_pose(self, owner, p_rel_m, q_rel, gripper_delta):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "pose_start"
                or request["future"].done()
            ):
                return False
            request["p_rel_m"] = list(p_rel_m)
            request["q_rel"] = list(q_rel)
            request["gripper_delta"] = float(gripper_delta)
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return True

    def pose_snapshot(self, request):
        with self.command_lock:
            if self.pending is not request or request["future"].done():
                return None
            return {
                "p_rel_m": list(request.get("p_rel_m", [0.0, 0.0, 0.0])),
                "q_rel": list(
                    request.get("q_rel", [0.0, 0.0, 0.0, 1.0])
                ),
                "gripper_delta": float(request.get("gripper_delta", 0.0)),
            }

    def request_pose_stop(self, owner):
        with self.command_lock:
            request = self.pending
            if (
                request is None
                or request.get("owner") is not owner
                or request["message"]["type"] != "pose_start"
                or request["future"].done()
            ):
                return "rejected", "pose_not_active"
            self.pose_stop_event.set()
            request["deadline"] = time.monotonic() + JOG_WATCHDOG_S
            return "ok", "pose_stop_requested"

    def clear_jog_stop(self):
        self.jog_stop_event.clear()

    def clear_joint_jog_stop(self):
        self.joint_jog_stop_event.clear()

    def clear_gripper_jog_stop(self):
        self.gripper_jog_stop_event.clear()

    def clear_orient_stop(self):
        self.orient_stop_event.clear()

    def clear_pose_stop(self):
        self.pose_stop_event.clear()

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
