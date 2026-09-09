#!/usr/bin/env python3
"""Send exactly one fixed +Z 2 cm command, then check measured base-frame motion."""
import argparse
import asyncio
import json
import math
import time

from websockets.legacy.client import connect


def message(kind, seq):
    return dict(v=0, type=kind, ts=int(time.time() * 1000), seq=seq, payload={})


def verify_result(payload):
    def vector(name):
        value = payload.get(name)
        if (not isinstance(value, list) or len(value) != 3
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
            raise ValueError(f"Missing/invalid measured vector: {name}")
        return value
    if payload.get("ok") is not True or payload.get("frame") != "base":
        raise ValueError(f"Simulation did not pass: {payload}")
    start, target, end = (vector(k) for k in ("start_m", "target_m", "end_m"))
    actual = [b - a for a, b in zip(start, end)]
    expected = [0.0, 0.0, 0.02]
    if math.dist([b - a for a, b in zip(start, target)], expected) > 1e-5:
        raise ValueError("Server target isn't base-frame +Z 2 cm")
    error = math.dist(actual, expected)
    if error > 0.005:
        raise ValueError(f"Measured displacement error {error:.6f} m exceeds 0.005 m")
    return start, target, end, actual, error


async def run(uri):
    async with connect(uri, open_timeout=10, close_timeout=3, max_size=8192) as ws:
        await ws.send(json.dumps(message("hello", 1)))
        welcome = json.loads(await asyncio.wait_for(ws.recv(), 10))
        config = welcome.get("payload", {})
        if (welcome.get("type") != "welcome" or welcome.get("seq") != 1
                or config.get("protocol") != "a3-smoke-1"
                or config.get("ready") is not True or config.get("used") is not False
                or config.get("frame") != "base"
                or config.get("delta_m") != [0.0, 0.0, 0.02]):
            raise RuntimeError(f"Wrong server or not ready: {welcome}")
        print("[WELCOME] A3 sim ready", flush=True)
        await ws.send(json.dumps(message("test_move_up", 2)))
        print("[SEND] base-frame +Z 0.020 m (once; no automatic retry)", flush=True)
        deadline = time.monotonic() + 60
        while True:
            reply = json.loads(await asyncio.wait_for(ws.recv(), max(0.01, deadline - time.monotonic())))
            if reply.get("seq") != 2:
                raise RuntimeError(f"Unexpected sequence: {reply}")
            if reply.get("type") == "ack":
                print("[ACK] accepted; waiting for measured sim result", flush=True)
                continue
            if reply.get("type") != "result":
                raise RuntimeError(f"Unexpected response: {reply}")
            start, target, end, actual, error = verify_result(reply["payload"])
            for label, value in (("start_m", start), ("target_m", target),
                                 ("end_m", end), ("actual_delta_m", actual)):
                print(f"{label}: {[round(x, 6) for x in value]}")
            print(f"position_error_m: {error:.6f}")
            print("A3 MAC -> SSH -> SO101 UP 2CM PASS")
            return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri))
    except Exception as exc:
        print(f"A3 FAIL: {exc}")
        print("Do not resend automatically. Inspect Server output; restart the smoke server for a fresh test.")
        raise SystemExit(1)
