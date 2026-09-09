#!/usr/bin/env python3
"""Reset an active A3/S4 emergency stop without issuing a movement."""

import argparse
import asyncio
import json
import time

from websockets.legacy.client import connect

from mac_move_delta import message, receive_json, wait_for_type


async def run(uri):
    async with connect(uri, open_timeout=10, close_timeout=3, max_size=8192) as ws:
        await ws.send(json.dumps(message("hello", 1, {
            "client": "phone",
            "name": "leo-mac-estop-reset",
        })))
        welcome = await receive_json(ws, 10)
        if welcome.get("type") != "welcome" or welcome.get("seq") != 1:
            raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
        print("[WELCOME] A3/S4 v0 server connected", flush=True)

        initial_state, _ = await wait_for_type(ws, "state", 1, time.monotonic() + 5)
        initial_estop = initial_state.get("payload", {}).get("estop")
        print(f"[STATE] estop={initial_estop}", flush=True)

        await ws.send(json.dumps(message("estop_reset", 2, {})))
        print("[SEND] estop_reset", flush=True)
        ack, _ = await wait_for_type(ws, "ack", 2, time.monotonic() + 5)
        payload = ack.get("payload", {})
        print(
            f"[ACK] status={payload.get('status')} reason={payload.get('reason')}",
            flush=True,
        )
        if (
            payload.get("seq") != 2
            or payload.get("status") != "ok"
            or payload.get("reason") != "estop_reset"
        ):
            raise RuntimeError(f"estop reset failed: {ack}")

        final_state, _ = await wait_for_type(ws, "state", 2, time.monotonic() + 5)
        final_estop = final_state.get("payload", {}).get("estop")
        if final_estop is not False:
            raise RuntimeError(f"state did not confirm estop=false: {final_state}")
        print("[STATE] estop=false", flush=True)
        print("A3/S4 ESTOP RESET PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri))
    except Exception as exc:
        print(f"A3/S4 ESTOP RESET FAIL: {exc}")
        raise SystemExit(1)
