#!/usr/bin/env python3
"""Verify that losing the control connection interrupts an in-flight move."""

import argparse
import asyncio
import json
import math
import time

from websockets.legacy.client import connect

from mac_estop import FREEZE_TOLERANCE_M, next_matching, next_state
from mac_move_delta import message, parse_state, receive_json


async def open_session(uri, name):
    ws = await connect(uri, open_timeout=10, close_timeout=3, max_size=8192)
    await ws.send(json.dumps(message("hello", 1, {
        "client": "phone",
        "name": name,
    })))
    welcome = await receive_json(ws, 10)
    if welcome.get("type") != "welcome" or welcome.get("seq") != 1:
        await ws.close()
        raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
    state, position = await next_state(ws, time.monotonic() + 5)
    return ws, state, position


async def run(uri):
    ws, _, start = await open_session(uri, "leo-mac-a3-disconnect-test")
    print(f"[WELCOME] start_m={[round(value, 6) for value in start]}", flush=True)
    try:
        await ws.send(json.dumps(message("move_delta", 2, {
            "axis": "z",
            "dir": 1,
            "step": "coarse",
        })))
        print("[SEND] start Z +20 mm, then drop the connection", flush=True)

        moving_position = None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            reply = await receive_json(ws, max(0.01, deadline - time.monotonic()))
            if reply.get("type") == "ack" and reply.get("seq") == 2:
                raise RuntimeError("move finished before disconnect could be tested")
            if reply.get("type") == "state":
                position = parse_state(reply)
                if (
                    reply.get("payload", {}).get("ik") == "moving"
                    and math.dist(start, position) >= 0.002
                ):
                    moving_position = position
                    break
        if moving_position is None:
            raise RuntimeError("did not observe an in-flight movement")
    finally:
        await ws.close()

    print(
        f"[DISCONNECT] EE had moved {math.dist(start, moving_position):.6f} m",
        flush=True,
    )
    await asyncio.sleep(0.5)

    ws, state, freeze_start = await open_session(
        uri, "leo-mac-a3-disconnect-reconnect"
    )
    try:
        payload = state.get("payload", {})
        if payload.get("estop") is not True or payload.get("ik") != "stopped":
            raise RuntimeError(f"disconnect did not engage estop: {state}")

        freeze_positions = [freeze_start]
        while len(freeze_positions) < 4:
            _, position = await next_state(ws, time.monotonic() + 3)
            freeze_positions.append(position)
        freeze_drift_m = max(
            math.dist(freeze_start, position) for position in freeze_positions[1:]
        )
        if freeze_drift_m > FREEZE_TOLERANCE_M:
            raise RuntimeError(
                f"EE drifted {freeze_drift_m:.6f} m after disconnect estop; "
                f"limit={FREEZE_TOLERANCE_M:.3f} m"
            )
        print(f"[FREEZE] max_drift_m={freeze_drift_m:.6f}", flush=True)

        await ws.send(json.dumps(message("move_delta", 2, {
            "axis": "x",
            "dir": 1,
            "step": "fine",
        })))
        blocked = await next_matching(
            ws,
            lambda item: item.get("type") == "ack" and item.get("seq") == 2,
            time.monotonic() + 5,
        )
        if blocked.get("payload", {}).get("reason") != "estop_active":
            raise RuntimeError(f"movement was not blocked after disconnect: {blocked}")
        print("[BLOCK] movement rejected while disconnect estop is active", flush=True)

        await ws.send(json.dumps(message("estop_reset", 3, {})))
        reset = await next_matching(
            ws,
            lambda item: item.get("type") == "ack" and item.get("seq") == 3,
            time.monotonic() + 5,
        )
        if (
            reset.get("payload", {}).get("status") != "ok"
            or reset.get("payload", {}).get("reason") != "estop_reset"
        ):
            raise RuntimeError(f"disconnect estop reset failed: {reset}")
        print("[RESET] estop=false", flush=True)
        print("A3 DISCONNECT ESTOP PASS")
    finally:
        await ws.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    arguments = parser.parse_args()
    try:
        asyncio.run(run(arguments.uri))
    except Exception as exc:
        print(f"A3 DISCONNECT ESTOP FAIL: {exc}")
        print("Do not retry automatically. Inspect the Server output first.")
        raise SystemExit(1)
