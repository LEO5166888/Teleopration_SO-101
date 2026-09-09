#!/usr/bin/env python3
"""Send one v0 gripper command and verify the returned simulator state."""

import argparse
import asyncio
import json
import time

from websockets.legacy.client import connect

from mac_move_delta import message, receive_json, wait_for_type


async def run(uri, action):
    async with connect(uri, open_timeout=10, close_timeout=3, max_size=8192) as ws:
        await ws.send(json.dumps(message("hello", 1, {
            "client": "phone",
            "name": "leo-mac-a3-gripper-test",
        })))
        welcome = await receive_json(ws, 10)
        if welcome.get("type") != "welcome" or welcome.get("seq") != 1:
            raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
        print("[WELCOME] A3 v0 server connected", flush=True)

        initial_state, _ = await wait_for_type(ws, "state", 1, time.monotonic() + 5)
        initial_gripper = initial_state.get("payload", {}).get("gripper")
        print(f"[STATE] gripper={initial_gripper}", flush=True)

        await ws.send(json.dumps(message("gripper", 2, {"action": action})))
        print(f"[SEND] gripper action={action}; no automatic retry", flush=True)
        ack, _ = await wait_for_type(ws, "ack", 2, time.monotonic() + 60)
        payload = ack.get("payload", {})
        status = payload.get("status")
        reason = payload.get("reason")
        print(f"[ACK] status={status} reason={reason}", flush=True)
        if payload.get("seq") != 2 or status not in {"ok", "clamped"}:
            raise RuntimeError(f"gripper command failed: {ack}")

        final_state, _ = await wait_for_type(ws, "state", 2, time.monotonic() + 5)
        final_gripper = final_state.get("payload", {}).get("gripper")
        if final_gripper != action:
            raise RuntimeError(
                f"state did not confirm action: expected={action}, actual={final_gripper}"
            )
        print(f"[STATE] gripper={final_gripper}")
        print(f"A3 GRIPPER {action.upper()} PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    parser.add_argument("--action", choices=("open", "close"), required=True)
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri, args.action))
    except Exception as exc:
        print(f"A3 GRIPPER FAIL: {exc}")
        print("Do not resend automatically. Inspect the Server output first.")
        raise SystemExit(1)
