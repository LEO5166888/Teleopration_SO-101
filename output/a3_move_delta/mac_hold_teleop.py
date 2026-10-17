#!/usr/bin/env python3
"""Press-and-hold Mac keyboard control for the S4 simulator.

The Tk window supplies real key-release events without global keyboard hooks.
Arm and gripper motion use start/keepalive/stop messages. Losing keepalives makes
the server stop.
"""

import argparse
import asyncio
import json
import tkinter as tk

from websockets.legacy.client import connect

from mac_move_delta import message


MOVES = {
    "a": ("x", 1, "x+ toward green target"),
    "d": ("x", -1, "x- away from green target"),
    "w": ("y", -1, "y- farther onto table"),
    "s": ("y", 1, "y+ toward robot base"),
    "r": ("z", 1, "z+ up"),
    "f": ("z", -1, "z- down"),
}

JOINT_JOGS = {
    "i": ("wrist_flex", 1, "wrist flex +"),
    "k": ("wrist_flex", -1, "wrist flex -"),
}

HELP = """Click this window, then hold a movement key:

W / S : farther onto table / toward robot base
A / D : toward green target / away from green target
R / F : up / down
Hold I / K: wrist flex + / -; release to stop

Release the key to stop.
1 / 2 : free fine 9 cm/s / coarse 18 cm/s
        closed-gripper carry: 3 cm/s / 6 cm/s
M     : toggle base/tool frame
Hold C: close gripper; release C: stop at the current opening
Hold O: open gripper; release O: stop at the current opening
      open=90/180 deg/s; close=30/60 deg/s
I / K : free=45/90 deg/s; carrying=15/30 deg/s
0     : reset scene
E / T : emergency stop / reset emergency stop
Q     : quit

Use fine + base near the table. Losing window focus also stops motion.
"""


class HoldTeleop:
    def __init__(self, root, ws):
        self.root = root
        self.ws = ws
        self.events = asyncio.Queue()
        self.pending = {}
        self.latest_state = {}
        self.state_event = asyncio.Event()
        self.seq = 2
        self.step = "fine"
        self.frame = "base"
        self.held_key = None
        self.release_timer = None
        self.down_discrete_keys = set()
        self.active_jog = None
        self.active_joint = None
        self.active_grip = None
        self.closed = False

        root.title("S4 SO-101 Hold Control")
        root.geometry("600x500")
        root.protocol("WM_DELETE_WINDOW", self.request_quit)
        root.bind("<KeyPress>", self.on_key_press)
        root.bind("<KeyRelease>", self.on_key_release)
        root.bind("<FocusOut>", self.on_focus_out)

        self.mode_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Connected. Click here before driving.")
        self.state_var = tk.StringVar(value="Waiting for state...")
        tk.Label(
            root,
            text="S4 SO-101 — hold to move, release to stop",
            font=("Helvetica", 18, "bold"),
        ).pack(pady=(16, 8))
        tk.Label(root, textvariable=self.mode_var, font=("Helvetica", 14)).pack()
        tk.Label(
            root,
            text=HELP,
            justify="left",
            font=("Menlo", 12),
        ).pack(padx=18, pady=10)
        tk.Label(
            root,
            textvariable=self.status_var,
            fg="#0057b8",
            wraplength=520,
            font=("Helvetica", 12, "bold"),
        ).pack(padx=18, pady=(0, 8))
        tk.Label(
            root,
            textvariable=self.state_var,
            wraplength=520,
            font=("Menlo", 10),
        ).pack(padx=18)
        self.refresh_mode()
        root.after(100, root.focus_force)

    def refresh_mode(self):
        self.mode_var.set(
            f"Mode: {self.step} / {self.frame}    "
            f"free={0.09 if self.step == 'fine' else 0.18:.2f} m/s; "
            f"carry={0.03 if self.step == 'fine' else 0.06:.2f} m/s"
        )

    def refresh_state(self):
        self.state_var.set(
            f"EE={self.latest_state.get('ee_pos')}\n"
            f"gripper={self.latest_state.get('gripper')}  "
            f"estop={self.latest_state.get('estop')}  "
            f"ik={self.latest_state.get('ik')}"
        )

    def enqueue(self, event):
        if not self.closed:
            self.events.put_nowait(event)

    def cancel_release_timer(self):
        if self.release_timer is not None:
            try:
                self.root.after_cancel(self.release_timer)
            except tk.TclError:
                pass
            self.release_timer = None

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in MOVES or key in JOINT_JOGS or key in {"c", "o"}:
            if self.held_key == key:
                # Tk may synthesize release/press pairs for key repeat.
                self.cancel_release_timer()
                return "break"
            if (
                self.held_key is not None
                or self.active_jog is not None
                or self.active_joint is not None
                or self.active_grip is not None
            ):
                return "break"
            self.held_key = key
            self.cancel_release_timer()
            self.enqueue(
                ("grip_start", key)
                if key in {"c", "o"}
                else (
                    ("joint_start", key)
                    if key in JOINT_JOGS
                    else ("jog_start", key)
                )
            )
            return "break"

        if key in self.down_discrete_keys:
            return "break"
        self.down_discrete_keys.add(key)
        self.enqueue(("key", key))
        return "break"

    def on_key_release(self, event):
        key = event.keysym.lower()
        self.down_discrete_keys.discard(key)
        if key == self.held_key:
            self.cancel_release_timer()
            self.release_timer = self.root.after(
                70,
                lambda released_key=key: self.finish_release(released_key),
            )
        return "break"

    def finish_release(self, key):
        self.release_timer = None
        if self.held_key == key:
            self.held_key = None
            self.enqueue(
                ("grip_stop", key)
                if key in {"c", "o"}
                else (
                    ("joint_stop", key)
                    if key in JOINT_JOGS
                    else ("jog_stop", key)
                )
            )

    def on_focus_out(self, _event):
        self.cancel_release_timer()
        if (
            self.held_key is not None
            or self.active_jog is not None
            or self.active_joint is not None
            or self.active_grip is not None
        ):
            stop_gripper = (
                self.held_key in {"c", "o"}
                or self.active_grip is not None
            )
            stop_joint = (
                self.held_key in JOINT_JOGS
                or self.active_joint is not None
            )
            self.held_key = None
            if stop_gripper:
                self.enqueue(("grip_stop", "focus_lost"))
            elif stop_joint:
                self.enqueue(("joint_stop", "focus_lost"))
            else:
                self.enqueue(("jog_stop", "focus_lost"))

    def request_quit(self):
        self.enqueue(("quit", None))

    async def send_with_ack(self, kind, payload, timeout=10):
        seq = self.seq
        self.seq += 1
        future = asyncio.get_running_loop().create_future()
        self.pending[seq] = future
        await self.ws.send(json.dumps(message(kind, seq, payload)))
        try:
            return await asyncio.wait_for(asyncio.shield(future), timeout)
        finally:
            self.pending.pop(seq, None)

    async def send_jog_start(self, key):
        axis, direction, label = MOVES[key]
        seq = self.seq
        self.seq += 1
        future = asyncio.get_running_loop().create_future()
        self.pending[seq] = future
        jog = {
            "key": key,
            "label": label,
            "seq": seq,
            "future": future,
            "heartbeat": None,
        }
        self.active_jog = jog
        await self.ws.send(json.dumps(message("jog_start", seq, {
            "axis": axis,
            "dir": direction,
            "step": self.step,
            "frame": self.frame,
        })))
        self.status_var.set(f"MOVING: {label} — release {key.upper()} to stop")
        print(f"[JOG START] {label} step={self.step} frame={self.frame}", flush=True)
        jog["heartbeat"] = asyncio.create_task(self.heartbeat_loop(jog))
        asyncio.create_task(self.monitor_jog(jog))

    async def heartbeat_loop(self, jog):
        try:
            while (
                self.active_jog is jog
                or self.active_joint is jog
                or self.active_grip is jog
            ):
                seq = self.seq
                self.seq += 1
                await self.ws.send(json.dumps(message("jog_keepalive", seq, {})))
                await asyncio.sleep(0.15)
        except asyncio.CancelledError:
            raise

    async def monitor_jog(self, jog):
        try:
            ack = await jog["future"]
        except Exception as exc:
            ack = {"payload": {"status": "rejected", "reason": str(exc)}}
        finally:
            self.pending.pop(jog["seq"], None)
        if self.active_jog is not jog:
            return
        self.active_jog = None
        self.held_key = None
        heartbeat = jog.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        payload = ack.get("payload", {})
        self.status_var.set(
            f"STOPPED: {payload.get('status')} / {payload.get('reason')}"
        )
        print(
            f"[JOG END] status={payload.get('status')} "
            f"reason={payload.get('reason')}",
            flush=True,
        )

    async def send_joint_jog(self, key):
        joint, direction, label = JOINT_JOGS[key]
        seq = self.seq
        self.seq += 1
        future = asyncio.get_running_loop().create_future()
        self.pending[seq] = future
        jog = {
            "key": key,
            "label": label,
            "seq": seq,
            "future": future,
            "heartbeat": None,
        }
        self.active_joint = jog
        await self.ws.send(json.dumps(message("joint_jog_start", seq, {
            "joint": joint,
            "dir": direction,
            "step": self.step,
        })))
        self.status_var.set(
            f"ROTATING: {label} — release {key.upper()} to stop"
        )
        print(
            f"[JOINT JOG START] {label} step={self.step}",
            flush=True,
        )
        jog["heartbeat"] = asyncio.create_task(self.heartbeat_loop(jog))
        asyncio.create_task(self.monitor_joint_jog(jog))

    async def monitor_joint_jog(self, jog):
        try:
            ack = await jog["future"]
        except Exception as exc:
            ack = {"payload": {"status": "rejected", "reason": str(exc)}}
        finally:
            self.pending.pop(jog["seq"], None)
        if self.active_joint is not jog:
            return
        self.active_joint = None
        self.held_key = None
        heartbeat = jog.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        payload = ack.get("payload", {})
        self.status_var.set(
            f"JOINT STOPPED: {payload.get('status')} / {payload.get('reason')}"
        )
        print(
            f"[JOINT JOG END] status={payload.get('status')} "
            f"reason={payload.get('reason')}",
            flush=True,
        )

    async def stop_joint_jog(self):
        jog = self.active_joint
        if jog is None:
            return
        self.active_joint = None
        heartbeat = jog.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        try:
            ack = await self.send_with_ack("joint_jog_stop", {}, timeout=3)
            payload = ack.get("payload", {})
            print(
                f"[JOINT JOG STOP] status={payload.get('status')} "
                f"reason={payload.get('reason')}",
                flush=True,
            )
            start_ack = await asyncio.wait_for(
                asyncio.shield(jog["future"]), 3
            )
            start_payload = start_ack.get("payload", {})
            self.status_var.set(
                f"JOINT STOPPED: {start_payload.get('status')} / "
                f"{start_payload.get('reason')}"
            )
        finally:
            self.pending.pop(jog["seq"], None)

    async def send_gripper_jog(self, key):
        action = "close" if key == "c" else "open"
        seq = self.seq
        self.seq += 1
        future = asyncio.get_running_loop().create_future()
        self.pending[seq] = future
        grip = {
            "key": key,
            "action": action,
            "seq": seq,
            "future": future,
            "heartbeat": None,
        }
        self.active_grip = grip
        await self.ws.send(json.dumps(message("gripper_jog_start", seq, {
            "action": action,
            "step": self.step,
        })))
        self.status_var.set(
            f"GRIPPER {action.upper()} — release {key.upper()} to stop"
        )
        print(
            f"[GRIPPER JOG START] action={action} step={self.step}",
            flush=True,
        )
        grip["heartbeat"] = asyncio.create_task(self.heartbeat_loop(grip))
        asyncio.create_task(self.monitor_grip(grip))

    async def monitor_grip(self, grip):
        try:
            ack = await grip["future"]
        except Exception as exc:
            ack = {"payload": {"status": "rejected", "reason": str(exc)}}
        finally:
            self.pending.pop(grip["seq"], None)
        if self.active_grip is not grip:
            return
        self.active_grip = None
        self.held_key = None
        heartbeat = grip.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        payload = ack.get("payload", {})
        self.status_var.set(
            f"GRIPPER: {payload.get('status')} / {payload.get('reason')}"
        )
        print(
            f"[GRIPPER JOG END] status={payload.get('status')} "
            f"reason={payload.get('reason')}",
            flush=True,
        )

    async def stop_gripper_jog(self):
        grip = self.active_grip
        if grip is None:
            return
        self.active_grip = None
        heartbeat = grip.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        try:
            ack = await self.send_with_ack("gripper_jog_stop", {}, timeout=3)
            payload = ack.get("payload", {})
            print(
                f"[GRIPPER JOG STOP] status={payload.get('status')} "
                f"reason={payload.get('reason')}",
                flush=True,
            )
            start_ack = await asyncio.wait_for(
                asyncio.shield(grip["future"]),
                3,
            )
            start_payload = start_ack.get("payload", {})
            self.status_var.set(
                f"GRIPPER STOPPED: {start_payload.get('status')} / "
                f"{start_payload.get('reason')}"
            )
        finally:
            self.pending.pop(grip["seq"], None)

    async def stop_jog(self):
        jog = self.active_jog
        if jog is None:
            return
        self.active_jog = None
        heartbeat = jog.get("heartbeat")
        if heartbeat is not None:
            heartbeat.cancel()
        try:
            ack = await self.send_with_ack("jog_stop", {}, timeout=3)
            payload = ack.get("payload", {})
            print(
                f"[JOG STOP] status={payload.get('status')} "
                f"reason={payload.get('reason')}",
                flush=True,
            )
            try:
                start_ack = await asyncio.wait_for(
                    asyncio.shield(jog["future"]), 3
                )
                start_payload = start_ack.get("payload", {})
                self.status_var.set(
                    f"STOPPED: {start_payload.get('status')} / "
                    f"{start_payload.get('reason')}"
                )
            except asyncio.TimeoutError:
                self.status_var.set("Stop sent; waiting for Server state")
        finally:
            self.pending.pop(jog["seq"], None)

    async def discrete_command(self, key):
        if (
            self.active_jog is not None
            or self.active_joint is not None
            or self.active_grip is not None
        ):
            self.status_var.set("Release the held key first")
            return
        if key == "1":
            new_step = "fine"
            self.step = new_step
            self.refresh_mode()
            ack = await self.send_with_ack(
                "mode", {"step": new_step}, timeout=10
            )
            result = ack.get("payload", {})
            self.status_var.set(
                f"mode: {result.get('status')} / {result.get('reason')}"
            )
            print(
                f"[mode] step={new_step} status={result.get('status')} "
                f"reason={result.get('reason')}",
                flush=True,
            )
            return
        if key == "2":
            new_step = "coarse"
            self.step = new_step
            self.refresh_mode()
            ack = await self.send_with_ack(
                "mode", {"step": new_step}, timeout=10
            )
            result = ack.get("payload", {})
            self.status_var.set(
                f"mode: {result.get('status')} / {result.get('reason')}"
            )
            print(
                f"[mode] step={new_step} status={result.get('status')} "
                f"reason={result.get('reason')}",
                flush=True,
            )
            return
        if key == "m":
            self.frame = "tool" if self.frame == "base" else "base"
            self.refresh_mode()
            return
        commands = {
            "0": ("reset_scene", {}),
            "e": ("estop", {}),
            "t": ("estop_reset", {}),
        }
        if key not in commands:
            return
        kind, payload = commands[key]
        self.status_var.set(f"Sending {kind}...")
        ack = await self.send_with_ack(kind, payload, timeout=60)
        result = ack.get("payload", {})
        self.status_var.set(
            f"{kind}: {result.get('status')} / {result.get('reason')}"
        )
        print(
            f"[{kind}] status={result.get('status')} "
            f"reason={result.get('reason')}",
            flush=True,
        )

    async def event_loop(self):
        while not self.closed:
            kind, value = await self.events.get()
            try:
                if kind == "jog_start":
                    await self.send_jog_start(value)
                elif kind == "jog_stop":
                    await self.stop_jog()
                elif kind == "joint_start":
                    await self.send_joint_jog(value)
                elif kind == "joint_stop":
                    await self.stop_joint_jog()
                elif kind == "grip_start":
                    await self.send_gripper_jog(value)
                elif kind == "grip_stop":
                    await self.stop_gripper_jog()
                elif kind == "key" and value == "q":
                    await self.stop_jog()
                    await self.stop_joint_jog()
                    await self.stop_gripper_jog()
                    self.closed = True
                elif kind == "key":
                    await self.discrete_command(value)
                elif kind == "quit":
                    await self.stop_jog()
                    await self.stop_joint_jog()
                    await self.stop_gripper_jog()
                    self.closed = True
                elif kind == "connection_error":
                    self.status_var.set(f"Connection failed: {value}")
                    self.closed = True
            except Exception as exc:
                self.status_var.set(f"ERROR: {exc}")
                print(f"S4 HOLD CONTROL ERROR: {exc}", flush=True)


async def receive_loop(controller):
    try:
        async for raw in controller.ws:
            reply = json.loads(raw)
            if reply.get("type") == "state":
                payload = reply.get("payload")
                if isinstance(payload, dict):
                    controller.latest_state.clear()
                    controller.latest_state.update(payload)
                    controller.state_event.set()
                    controller.refresh_state()
                continue
            seq = reply.get("seq")
            future = controller.pending.get(seq)
            if future is not None and not future.done():
                future.set_result(reply)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        controller.enqueue(("connection_error", exc))


async def run(uri):
    async with connect(
        uri,
        open_timeout=10,
        close_timeout=3,
        max_size=8192,
        ping_interval=20,
        ping_timeout=20,
    ) as ws:
        await ws.send(json.dumps(message("hello", 1, {
            "client": "phone",
            "name": "leo-mac-s4-hold-keyboard",
        })))
        welcome = json.loads(await asyncio.wait_for(ws.recv(), 10))
        if welcome.get("type") != "welcome":
            raise RuntimeError(f"wrong server or invalid welcome: {welcome}")

        root = tk.Tk()
        controller = HoldTeleop(root, ws)
        receiver = asyncio.create_task(receive_loop(controller))
        event_worker = asyncio.create_task(controller.event_loop())
        print("S4 HOLD CONTROL READY — use the focused control window", flush=True)
        try:
            while not controller.closed:
                root.update()
                await asyncio.sleep(0.01)
        finally:
            controller.closed = True
            receiver.cancel()
            event_worker.cancel()
            await asyncio.gather(receiver, event_worker, return_exceptions=True)
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:8766")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.uri))
    except (KeyboardInterrupt, tk.TclError):
        print("\nS4 hold control closed")
    except Exception as exc:
        print(f"S4 HOLD CONTROL FAIL: {exc}")
        raise SystemExit(1)
