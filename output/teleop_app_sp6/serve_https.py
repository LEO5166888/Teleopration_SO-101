"""HTTPS 靜態頁伺服器＋wss 轉接（同一個 port 8443）。

手機的方向感測器（DeviceOrientation、AbsoluteOrientationSensor）只給 HTTPS 頁面用，
所以姿態模式與 orientation.html 不能用 http.server 開。第一次執行會用 openssl 產自簽憑證
（.cert/ 目錄，已加進 .gitignore）；手機瀏覽器會警告憑證不受信任，按「仍要前往」。

HTTPS 頁面不能連 ws://（瀏覽器擋 mixed content），所以同一個 port 再開一條 wss://<IP>:8443/relay，
把 App 的訊息原封不動轉給 --upstream（預設本機 stub），回來的也原封不動轉回。
跟頁面同 port，憑證只要在手機上按一次「仍要前往」。

    python3 serve_https.py                                   # 轉給本機 stub
    python3 serve_https.py --upstream ws://192.168.50.13:8765  # 轉給 Leo 的 server
"""

import argparse
import asyncio
import http
import mimetypes
import pathlib
import ssl
import subprocess

import websockets
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

PORT = 8443
ROOT = pathlib.Path(__file__).parent.resolve()
CERT_DIR = ROOT / ".cert"
CERT, KEY = CERT_DIR / "cert.pem", CERT_DIR / "key.pem"
RELAY_PATH = "/relay"


def ensure_cert():
    if CERT.exists() and KEY.exists():
        return
    CERT_DIR.mkdir(exist_ok=True)
    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-keyout",
            str(KEY),
            "-out",
            str(CERT),
            "-days",
            "365",
            "-subj",
            "/CN=teleop-app-dev",
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"已產生自簽憑證：{CERT_DIR}")


def respond(status, body, content_type="text/plain; charset=utf-8"):
    headers = Headers(
        [
            ("Content-Type", content_type),
            ("Content-Length", str(len(body))),
            ("Cache-Control", "no-store"),  # 改完 app.js 重新整理就生效
        ]
    )
    return Response(status.value, status.phrase, headers, body)


def static_file(path):
    """回傳 ROOT 底下的檔案；不出 ROOT、不給隱藏檔（.cert 裡有私鑰）。"""
    rel = path.split("?", 1)[0].lstrip("/") or "index.html"
    target = (ROOT / rel).resolve()
    if (
        not target.is_relative_to(ROOT)
        or any(part.startswith(".") for part in target.relative_to(ROOT).parts)
        or not target.is_file()
    ):
        return respond(http.HTTPStatus.NOT_FOUND, b"not found")
    ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
    if ctype.startswith("text/") or ctype == "application/javascript":
        ctype += "; charset=utf-8"
    return respond(http.HTTPStatus.OK, target.read_bytes(), ctype)


def process_request(connection, request):
    # 只有打 /relay 而且要升級成 WebSocket 的請求往下走握手，其餘當靜態檔
    is_upgrade = "websocket" in request.headers.get("Upgrade", "").lower()
    if request.path.split("?", 1)[0] == RELAY_PATH and is_upgrade:
        return None
    return static_file(request.path)


async def pump(src, dst):
    try:
        async for msg in src:
            await dst.send(msg)
    except websockets.ConnectionClosed:
        pass


def make_relay(upstream):
    async def relay(client):
        peer = client.remote_address[0] if client.remote_address else "?"
        try:
            async with connect(upstream) as server:
                print(f"relay 接通：{peer} ⇄ {upstream}")
                tasks = [
                    asyncio.create_task(pump(client, server)),
                    asyncio.create_task(pump(server, client)),
                ]
                # 任一邊斷就兩邊都斷：App 會看到斷線並自動重連，server 端照它自己的規則處理
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for t in tasks:
                    t.cancel()
        except (OSError, websockets.InvalidHandshake) as e:
            print(f"relay 連不到 {upstream}：{e}")
        except websockets.ConnectionClosed:
            pass
        finally:
            print(f"relay 結束：{peer}")

    return relay


async def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--upstream", default="ws://127.0.0.1:8765")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    ensure_cert()
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(str(CERT), str(KEY))
    async with serve(
        make_relay(args.upstream),
        "0.0.0.0",
        args.port,
        ssl=ctx,
        process_request=process_request,
    ):
        print(f"App：      https://<電腦區網 IP>:{args.port}/")
        print(f"姿態 demo：https://<電腦區網 IP>:{args.port}/orientation.html")
        print(
            f"wss 轉接： wss://<電腦區網 IP>:{args.port}{RELAY_PATH} → {args.upstream}"
        )
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

