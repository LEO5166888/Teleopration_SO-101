#!/usr/bin/env python3
"""Interrupt an in-flight move_delta with estop on the same WebSocket."""

import argparse
import asyncio
import json
import math
import time

from websockets.legacy.client import connect

from mac_move_delta import message, parse_state, receive_json


FREEZE_TOLERANCE_M = 0.002


async def next_matching(ws, predicate, deadline):
    while time.monotonic() < deadline:
        reply = await receive_json(ws, max(0.01, deadline - time.monotonic()))
        if reply.get("type") == "error":
            raise RuntimeError(f"server error: {reply}")
        if predicate(reply):
            return reply
    raise TimeoutError("expected response was not received")


async def next_state(ws, deadline):
    reply = await next_matching(
        ws,
        lambda item: item.get("type") == "state",
        deadline,
    )
    position = parse_state(reply)
    return reply, position


async def run(uri):
    async with connect(uri, open_timeout=10, close_timeout=3, max_size=8192) as ws:
        await ws.send(json.dumps(message("hello", 1, {
            "client": "phone",
            "name": "leo-mac-a3-estop-test",
        })))
        welcome = await receive_json(ws, 10)
        if welcome.get("type") != "welcome" or welcome.get("seq") != 1:
            raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
        _, start = await next_state(ws, time.monotonic() + 5)
        print(f"[WELCOME] start_m={[round(value, 6) for value in start]}", flush=True)

        await ws.send(json.dumps(message("move_delta", 2, {
            "axis": "z",
            "dir": 1,
            "step": "coarse",
        })))
        print("[SEND] start Z +20 mm, then interrupt while moving", flush=True)

        moving_position = None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            reply = await receive_json(ws, max(0.01, deadline - time.monotonic()))
            if reply.get("type") == "ack" and reply.get("seq") == 2:
                raise RuntimeError("move finished before estop could be tested")
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

        await ws.send(json.dumps(message("estop", 3, {})))
        print(
            f"[SEND] estop while EE had moved {math.dist(start, moving_position):.6f} m",
            flush=True,
        )

        got_estop_ack = False
        got_move_rejection = False
        got_estop_state = False
        deadline = time.monotonic() + 10
        while not (got_estop_ack and got_move_rejection and got_estop_state):
            reply = await receive_json(ws, max(0.01, deadline - time.monotonic()))
            if reply.get("type") == "ack" and reply.get("seq") == 3:
                payload = reply.get("payload", {})
                got_estop_ack = (
                    payload.get("status") == "ok"
                    and payload.get("reason") == "estop_engaged"
                )
            elif reply.get("type") == "ack" and reply.get("seq") == 2:
                payload = reply.get("payload", {})
                got_move_rejection = (
                    payload.get("status") == "rejected"
                    and payload.get("reason") == "estop"
                )
            elif reply.get("type") == "state":
                parse_state(reply)
                got_estop_state = reply.get("payload", {}).get("estop") is True

        _, freeze_start = await next_state(ws, time.monotonic() + 3)
        freeze_positions = [freeze_start]
        while len(freeze_positions) < 4:
            _, position = await next_state(ws, time.monotonic() + 3)
            freeze_positions.append(position)
        freeze_drift_m = max(
            math.dist(freeze_start, position) for position in freeze_positions[1:]
        )
        if freeze_drift_m > FREEZE_TOLERANCE_M:
            raise RuntimeError(
                f"EE drifted {freeze_drift_m:.6f} m after estop; "
                f"limit={FREEZE_TOLERANCE_M:.3f} m"
            )
        print(f"[FREEZE] max_drift_m={freeze_drift_m:.6f}", flush=True)

        await ws.send(json.dumps(message("move_delta", 4, {
            "axis": "x",
            "dir": 1,
            "step": "fine",
        })))
        blocked = await next_matching(
            ws,
            lambda item: item.get("type") == "ack" and item.get("seq") == 4,
            time.monotonic() + 5,
        )
        if blocked.get("payload", {}).get("reason") != "estop_active":
            raise RuntimeError(f"command was not blocked during estop: {blocked}")
        print("[BLOCK] movement rejected while estop is active", flush=True)

        await ws.send(json.dumps(message("estop_reset", 5, {})))
        reset_ack = await next_matching(
            ws,
            lambda item: item.get("type") == "ack" and item.get("seq") == 5,
            time.monotonic() + 5,
        )
        if (
            reset_ack.get("payload", {}).get("status") != "ok"
            or reset_ack.get("payload", {}).get("reason") != "estop_reset"
        ):
            raise RuntimeError(f"estop reset failed: {reset_ack}")
        reset_state = await next_matching(
            ws,
            lambda item: (
                item.get("type") == "state"
                and item.get("payload", {}).get("estop") is False
            ),
            time.monotonic() + 5,
        )
        parse_state(reset_state)
        print("[RESET] estop=false", flush=True)
        print("A3 ESTOP SAME-CONNECTION INTERRUPT PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri))
    except Exception as exc:
        print(f"A3 ESTOP FAIL: {exc}")
        print("Do not retry automatically. Inspect the Server output first.")
        raise SystemExit(1)
