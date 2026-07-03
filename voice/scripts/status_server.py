"""Test status server for Purple's face UI.

Watches a markdown file and broadcasts its contents to WebSocket clients as
`{"markdown": <string|null>}`. Empty or missing file -> null (face returns
to fullscreen). Any writer that edits the file drives the display.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
from typing import Optional

import websockets


async def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--file", default="status.md")
    parser.add_argument("--poll", type=float, default=0.5)
    args = parser.parse_args()

    watch_path = Path(args.file).resolve()
    clients: set = set()
    current: Optional[str] = None

    def read_content() -> Optional[str]:
        try:
            content = watch_path.read_text().strip()
        except FileNotFoundError:
            return None
        return content if content else None

    async def broadcast(payload: dict) -> None:
        if not clients:
            return
        data = json.dumps(payload)
        await asyncio.gather(
            *(ws.send(data) for ws in list(clients)),
            return_exceptions=True,
        )

    async def handler(ws):
        clients.add(ws)
        try:
            await ws.send(json.dumps({"markdown": current}))
            await ws.wait_closed()
        finally:
            clients.discard(ws)

    async def watch_loop() -> None:
        nonlocal current
        last_mtime = -1.0
        while True:
            try:
                mtime = os.path.getmtime(watch_path)
            except FileNotFoundError:
                mtime = 0.0
            if mtime != last_mtime:
                last_mtime = mtime
                new = read_content()
                if new != current:
                    current = new
                    label = "null" if current is None else f"{len(current)} chars"
                    print(f"[status] broadcast: {label}")
                    await broadcast({"markdown": current})
            await asyncio.sleep(args.poll)

    server = await websockets.serve(handler, args.host, args.port)
    print(f"[status] ws://{args.host}:{args.port}  watching {watch_path}")
    try:
        await watch_loop()
    finally:
        server.close()
        await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(main())
