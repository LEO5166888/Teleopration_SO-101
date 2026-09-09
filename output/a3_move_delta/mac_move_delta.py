#!/usr/bin/env python3
"""Send one formal v0 move_delta and verify the measured base-frame motion."""

import argparse
import asyncio
import json
import math
import time

from websockets.legacy.client import connect


STEP_METERS = {"coarse": 0.020, "fine": 0.005}
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
TOLERANCE_M = {"coarse": 0.005, "fine": 0.0015}


def message(kind, seq, payload):
    return {
        "v": 0,
        "type": kind,
        "ts": int(time.time() * 1000),
        "seq": seq,
        "payload": payload,
    }


def parse_state(reply):
    if reply.get("type") != "state":
        return None
    payload = reply.get("payload")
    if not isinstance(payload, dict):
        raise ValueError(f"invalid state payload: {reply}")
    ee_pos = payload.get("ee_pos")
    if (
        not isinstance(ee_pos, list)
        or len(ee_pos) != 3
        or any(type(value) not in (int, float) or not math.isfinite(value) for value in ee_pos)
    ):
        raise ValueError(f"invalid state.ee_pos: {reply}")
    return ee_pos


async def receive_json(ws, timeout):
    raw = await asyncio.wait_for(ws.recv(), timeout)
    reply = json.loads(raw)
    if not isinstance(reply, dict):
        raise ValueError(f"reply is not an object: {reply}")
    return reply


async def wait_for_type(ws, expected_type, seq, deadline):
    latest_state = None
    while time.monotonic() < deadline:
        reply = await receive_json(ws, max(0.01, deadline - time.monotonic()))
        if reply.get("type") == "state":
            latest_state = parse_state(reply)
            if expected_type == "state" and reply.get("seq") == seq:
                return reply, latest_state
            continue
        if reply.get("type") == "error":
            raise RuntimeError(f"server error: {reply}")
        if reply.get("type") == expected_type and reply.get("seq") == seq:
            return reply, latest_state
        raise RuntimeError(f"unexpected reply: {reply}")
    raise TimeoutError(f"timed out waiting for {expected_type}")


def verify_motion(start, end, axis, direction, step_name, status):
    actual = [finish - begin for begin, finish in zip(start, end)]
    expected = [0.0, 0.0, 0.0]
    expected[AXIS_INDEX[axis]] = direction * STEP_METERS[step_name]
    error = math.dist(actual, expected)
    tolerance_m = TOLERANCE_M[step_name]
    if status == "ok" and error > tolerance_m:
        raise ValueError(
            f"measured displacement error {error:.6f} m exceeds {tolerance_m:.4f} m"
        )
    if status == "clamped":
        moved = actual[AXIS_INDEX[axis]]
        if direction * moved < -0.001:
            raise ValueError("clamped command moved in the opposite direction")
    return actual, expected, error


async def run(uri, axis, direction, step_name):
    async with connect(uri, open_timeout=10, close_timeout=3, max_size=8192) as ws:
        await ws.send(json.dumps(message("hello", 1, {
            "client": "phone",
            "name": "leo-mac-a3-test",
        })))
        welcome = await receive_json(ws, 10)
        if (
            welcome.get("type") != "welcome"
            or welcome.get("seq") != 1
            or welcome.get("payload", {}).get("config")
            != {"coarse_cm": 2.0, "fine_cm": 0.5}
        ):
            raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
        print("[WELCOME] A3 v0 server connected", flush=True)

        _, start = await wait_for_type(ws, "state", 1, time.monotonic() + 5)
        if start is None:
            raise RuntimeError("simulation state is not ready")

        payload = {"axis": axis, "dir": direction, "step": step_name}
        await ws.send(json.dumps(message("move_delta", 2, payload)))
        signed_step = direction * STEP_METERS[step_name]
        print(
            f"[SEND] move_delta axis={axis} dir={direction:+d} "
            f"step={step_name} ({signed_step:+.3f} m); no automatic retry",
            flush=True,
        )

        ack, latest_state = await wait_for_type(ws, "ack", 2, time.monotonic() + 60)
        ack_payload = ack.get("payload", {})
        status = ack_payload.get("status")
        reason = ack_payload.get("reason")
        if ack_payload.get("seq") != 2 or status not in {"ok", "clamped", "rejected"}:
            raise RuntimeError(f"invalid ack: {ack}")
        print(f"[ACK] status={status} reason={reason}", flush=True)
        if status == "rejected":
            raise RuntimeError(f"simulation rejected command: {reason}")

        state_after_ack, post_ack_state = await wait_for_type(
            ws, "state", 2, time.monotonic() + 5
        )
        end = parse_state(state_after_ack)
        if end is None:
            end = post_ack_state or latest_state
        if end is None:
            raise RuntimeError("no final state received after ack")

        actual, expected, error = verify_motion(
            start, end, axis, direction, step_name, status
        )
        print(f"start_m: {[round(value, 6) for value in start]}")
        print(f"end_m: {[round(value, 6) for value in end]}")
        print(f"expected_delta_m: {[round(value, 6) for value in expected]}")
        print(f"actual_delta_m: {[round(value, 6) for value in actual]}")
        print(f"position_error_m: {error:.6f}")
        print(
            f"A3 MOVE_DELTA {axis.upper()} {'PLUS' if direction > 0 else 'MINUS'} "
            f"{step_name.upper()} PASS"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    parser.add_argument("--axis", choices=("x", "y", "z"), required=True)
    parser.add_argument("--dir", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--step", choices=("coarse", "fine"), default="coarse")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri, args.axis, args.dir, args.step))
    except Exception as exc:
        print(f"A3 MOVE_DELTA FAIL: {exc}")
        print("Do not resend automatically. Inspect the Server output first.")
        raise SystemExit(1)
