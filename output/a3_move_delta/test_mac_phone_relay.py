#!/usr/bin/env python3
"""Local integration check for the transparent Mac phone relay."""

import asyncio

from websockets.legacy.client import connect
from websockets.legacy.server import serve

from mac_phone_relay import PhoneRelay


PHONE_MESSAGE = (
    '{"v":0,"type":"move_delta","ts":123,"seq":7,'
    '"payload":{"axis":"z","dir":1,"step":"coarse"}}'
)
SERVER_MESSAGE = (
    '{"v":0,"type":"ack","ts":124,"seq":7,'
    '"payload":{"seq":7,"status":"ok","reason":"completed"}}'
)


async def main():
    received_by_server = []

    async def fake_rtxws(websocket):
        received_by_server.append(await websocket.recv())
        await websocket.send(SERVER_MESSAGE)

    async with serve(fake_rtxws, "127.0.0.1", 0) as upstream_server:
        upstream_port = upstream_server.sockets[0].getsockname()[1]
        relay = PhoneRelay(f"ws://127.0.0.1:{upstream_port}", log_file=None)
        async with serve(relay.handler, "127.0.0.1", 0) as relay_server:
            relay_port = relay_server.sockets[0].getsockname()[1]
            async with connect(f"ws://127.0.0.1:{relay_port}") as phone:
                await phone.send(PHONE_MESSAGE)
                received_by_phone = await asyncio.wait_for(phone.recv(), 2)

    assert received_by_server == [PHONE_MESSAGE]
    assert received_by_phone == SERVER_MESSAGE
    print("MAC PHONE RELAY LOCAL ROUND-TRIP PASS")


if __name__ == "__main__":
    asyncio.run(main())
