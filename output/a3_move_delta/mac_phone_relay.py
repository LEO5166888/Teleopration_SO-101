#!/usr/bin/env python3
"""Transparent WebSocket relay from Thomas's phone App to the RTXWS tunnel.

Phone-facing side:
    ws://<Mac LAN IP>:8765

RTXWS-facing side:
    ws://127.0.0.1:8766 (the local end of the user's SSH tunnel)

The relay never stores an SSH password, buffers commands, or retries a command.
If either connection closes, the paired connection is closed as well.
"""

import argparse
import asyncio
import json
import socket
import subprocess
import time
from pathlib import Path

from websockets.exceptions import ConnectionClosed
from websockets.legacy.client import connect
from websockets.legacy.server import serve


def detect_lan_ip():
    """Return the Mac Wi-Fi IPv4 address, with a routed-address fallback."""
    try:
        result = subprocess.run(
            ["ipconfig", "getifaddr", "en0"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
        address = result.stdout.strip()
        if result.returncode == 0 and address:
            return address
    except (OSError, subprocess.TimeoutExpired):
        pass

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


class PhoneRelay:
    def __init__(self, upstream_uri, log_file):
        self.upstream_uri = upstream_uri
        self.log_file = Path(log_file) if log_file else None
        self.active_phone = asyncio.Lock()

    def log(self, direction, peer, raw_message):
        if self.log_file is None:
            return
        record = {
            "relay_ts": int(time.time() * 1000),
            "direction": direction,
            "peer": peer,
            "message": raw_message,
        }
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        with self.log_file.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    async def forward(self, source, destination, direction, peer):
        async for raw_message in source:
            if not isinstance(raw_message, str):
                await source.close(code=1003, reason="text JSON messages only")
                return
            self.log(direction, peer, raw_message)
            await destination.send(raw_message)

    async def handler(self, phone):
        peer = str(phone.remote_address)
        if self.active_phone.locked():
            await phone.close(code=1013, reason="another App is already connected")
            print(f"[REJECT] second App connection peer={peer}", flush=True)
            return

        async with self.active_phone:
            print(f"[PHONE CONNECT] peer={peer}", flush=True)
            upstream = None
            upstream_failed = False
            try:
                upstream = await connect(
                    self.upstream_uri,
                    open_timeout=5,
                    close_timeout=2,
                    max_size=8192,
                    max_queue=4,
                )
                print(f"[RTXWS CONNECT] {self.upstream_uri}", flush=True)

                phone_to_server = asyncio.create_task(
                    self.forward(phone, upstream, "phone_to_rtxws", peer)
                )
                server_to_phone = asyncio.create_task(
                    self.forward(upstream, phone, "rtxws_to_phone", peer)
                )
                done, pending = await asyncio.wait(
                    {phone_to_server, server_to_phone},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in pending:
                    task.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
                for task in done:
                    try:
                        task.result()
                    except ConnectionClosed:
                        pass
            except (OSError, asyncio.TimeoutError) as exc:
                upstream_failed = True
                print(f"[RTXWS UNAVAILABLE] {exc}", flush=True)
            except ConnectionClosed:
                pass
            finally:
                if upstream is not None:
                    await upstream.close()
                if not phone.closed:
                    await phone.close(
                        code=1011,
                        reason=(
                            "RTXWS tunnel unavailable"
                            if upstream_failed
                            else "paired connection closed"
                        ),
                    )
                print(f"[PHONE DISCONNECT] peer={peer}", flush=True)


async def run(args):
    relay = PhoneRelay(args.upstream, args.log_file)
    async with serve(
        relay.handler,
        args.listen_host,
        args.listen_port,
        max_size=8192,
        max_queue=4,
        close_timeout=2,
    ) as server:
        actual_port = server.sockets[0].getsockname()[1]
        lan_ip = detect_lan_ip()
        print(
            f"MAC PHONE RELAY READY listen=ws://{args.listen_host}:{actual_port} "
            f"upstream={args.upstream}",
            flush=True,
        )
        if lan_ip:
            print(f"Thomas App URL: ws://{lan_ip}:{actual_port}", flush=True)
        else:
            print(
                f"Thomas App URL: ws://<MAC_LAN_IP>:{actual_port}", flush=True
            )
        print("One App connection; no command buffering or automatic retry.", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8765)
    parser.add_argument("--upstream", default="ws://127.0.0.1:8766")
    parser.add_argument(
        "--log-file",
        default=str(Path(__file__).with_name("mac_phone_relay.jsonl")),
    )
    arguments = parser.parse_args()
    try:
        asyncio.run(run(arguments))
    except KeyboardInterrupt:
        print("\nMAC PHONE RELAY STOPPED", flush=True)
