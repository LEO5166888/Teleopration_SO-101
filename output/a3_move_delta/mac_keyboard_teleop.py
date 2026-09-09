#!/usr/bin/env python3
"""Simple one-key-at-a-time terminal control for the A3/S4 simulator."""

import argparse
import asyncio
import json
import select
import sys

from websockets.legacy.client import connect

from mac_move_delta import message


MOVES = {
    "a": ("x", 1, "toward green target"),
    "d": ("x", -1, "away from green target"),
    "w": ("y", -1, "farther onto table"),
    "s": ("y", 1, "toward robot base"),
    "r": ("z", 1, "up"),
    "f": ("z", -1, "down"),
}
JOINT_MOVES = {
    "i": ("wrist_flex", 1, "wrist flex +"),
    "k": ("wrist_flex", -1, "wrist flex -"),
}


def show_help():
    print(
        """
S4 MAC KEYBOARD CONTROL (type one key, then Enter)
  a : x+  toward green target     d : x-  opposite direction
  w : y-  farther onto table      s : y+  toward robot base
  r : z+  up                      f : z-  down
  i : wrist flex +                k : wrist flex -
  o : open gripper                c : close gripper
  1 : fine 0.5 cm                 2 : coarse 2.0 cm
  m : toggle base/tool frame
  0 : reset robot, gripper, and cube to the initial scene
  e : emergency stop              t : reset emergency stop
  h : show this help              q : quit

Use fine mode near the table. One entry sends exactly one command; no retry.
For a vertical lift, use base mode and press r before any horizontal movement.
""".strip()
    )


def discard_buffered_input():
    """Drop complete lines typed while the previous command was executing."""
    discarded = 0
    while select.select([sys.stdin], [], [], 0)[0]:
        if not sys.stdin.readline():
            break
        discarded += 1
    if discarded:
        print(
            f"[IGNORED] discarded {discarded} buffered input line(s); "
            "wait for the next S4 prompt before typing.",
            flush=True,
        )


async def receive_loop(ws, control_queue, latest_state, state_event):
    """Continuously drain broadcasts so the WebSocket cannot stall while idle."""
    try:
        async for raw in ws:
            reply = json.loads(raw)
            if not isinstance(reply, dict):
                raise RuntimeError(f"reply is not an object: {reply}")
            if reply.get("type") == "state":
                payload = reply.get("payload")
                if isinstance(payload, dict):
                    latest_state.clear()
                    latest_state.update(payload)
                    state_event.set()
                continue
            await control_queue.put(reply)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        await control_queue.put(exc)


async def wait_for_control(control_queue, expected_type, seq, timeout_s):
    async def next_match():
        while True:
            item = await control_queue.get()
            if isinstance(item, Exception):
                raise item
            if item.get("type") == "error":
                raise RuntimeError(f"server error: {item}")
            if item.get("type") == expected_type and item.get("seq") == seq:
                return item

    return await asyncio.wait_for(next_match(), timeout_s)


async def print_state_after_command(latest_state, state_event):
    try:
        await asyncio.wait_for(state_event.wait(), 1.0)
    except TimeoutError:
        pass
    print(
        f"[STATE] ee_pos={latest_state.get('ee_pos')} "
        f"gripper={latest_state.get('gripper')} "
        f"estop={latest_state.get('estop')} ik={latest_state.get('ik')}",
        flush=True,
    )


async def run(uri):
    async with connect(
        uri,
        open_timeout=10,
        close_timeout=3,
        max_size=8192,
        ping_interval=20,
        ping_timeout=20,
    ) as ws:
        control_queue = asyncio.Queue()
        latest_state = {}
        state_event = asyncio.Event()
        receiver = asyncio.create_task(
            receive_loop(ws, control_queue, latest_state, state_event)
        )
        try:
            await ws.send(json.dumps(message("hello", 1, {
                "client": "phone",
                "name": "leo-mac-s4-keyboard",
            })))
            welcome = await wait_for_control(control_queue, "welcome", 1, 10)
            if welcome.get("type") != "welcome":
                raise RuntimeError(f"wrong server or invalid welcome: {welcome}")
            await asyncio.wait_for(state_event.wait(), 5)
            print(
                f"[CONNECTED] estop={latest_state.get('estop')} "
                f"gripper={latest_state.get('gripper')}",
                flush=True,
            )

            step = "fine"
            frame = "base"
            seq = 2
            show_help()
            while True:
                key = (
                    await asyncio.to_thread(input, f"\nS4[{step}/{frame}]> ")
                ).strip().lower()
                if not key:
                    continue
                if key == "q":
                    print("S4 keyboard control closed")
                    return
                if key == "h":
                    show_help()
                    continue
                if key in {"1", "2"}:
                    step = "fine" if key == "1" else "coarse"
                    print(f"[MODE] {step}")
                    continue
                if key == "m":
                    frame = "tool" if frame == "base" else "base"
                    print(f"[FRAME] {frame}")
                    continue

                if key in MOVES:
                    axis, direction, label = MOVES[key]
                    kind = "move_delta"
                    payload = {
                        "axis": axis,
                        "dir": direction,
                        "step": step,
                        "frame": frame,
                    }
                    print(
                        f"[SEND] {axis}{'+' if direction > 0 else '-'} "
                        f"{step} frame={frame}: "
                        f"{label if frame == 'base' else 'tool-local direction'}",
                        flush=True,
                    )
                    timeout_s = 60
                elif key in JOINT_MOVES:
                    joint, direction, label = JOINT_MOVES[key]
                    kind = "joint_delta"
                    payload = {"joint": joint, "dir": direction, "step": step}
                    print(
                        f"[SEND] {label} {step}",
                        flush=True,
                    )
                    timeout_s = 60
                elif key in {"o", "c"}:
                    action = "open" if key == "o" else "close"
                    kind = "gripper"
                    payload = {"action": action}
                    print(f"[SEND] gripper {action}", flush=True)
                    timeout_s = 60
                elif key in {"e", "t"}:
                    kind = "estop" if key == "e" else "estop_reset"
                    payload = {}
                    print(f"[SEND] {kind}", flush=True)
                    timeout_s = 5
                elif key == "0":
                    kind = "reset_scene"
                    payload = {}
                    print("[SEND] reset_scene", flush=True)
                    timeout_s = 10
                else:
                    print("Unknown key. Type h for help.")
                    continue

                state_event.clear()
                await ws.send(json.dumps(message(kind, seq, payload)))
                ack = await wait_for_control(control_queue, "ack", seq, timeout_s)
                ack_payload = ack.get("payload", {})
                status = ack_payload.get("status")
                reason = ack_payload.get("reason")
                print(f"[ACK] status={status} reason={reason}", flush=True)
                await print_state_after_command(latest_state, state_event)

                if kind == "gripper" and payload["action"] == "close" and status == "ok":
                    print("[NEXT] Switch to base mode if needed, then press r to lift.")
                if status == "rejected":
                    print(
                        "[STOP] Command rejected. Do not repeat it. "
                        "Use t only if estop=true."
                    )
                discard_buffered_input()
                seq += 1
        finally:
            receiver.cancel()
            await asyncio.gather(receiver, return_exceptions=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri))
    except (EOFError, KeyboardInterrupt):
        print("\nS4 keyboard control closed")
    except Exception as exc:
        print(f"S4 KEYBOARD CONTROL FAIL: {exc}")
        raise SystemExit(1)
