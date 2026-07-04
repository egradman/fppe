"""Grab a single still frame from the Pi's MJPEG camera stream.

viser_control (fppe:8091) serves `multipart/x-mixed-replace` JPEG streams at
`/mjpeg/<label>` (cam0, cam1). We don't need a video; we just read bytes until
the first complete JPEG (SOI 0xFFD8 ... EOI 0xFFD9) and decode it with Pillow.
"""
from __future__ import annotations

import io
import urllib.request

_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"
_MAX_BYTES = 2_000_000  # a 320x240 q60 JPEG is a few KB; cap well above one frame


def grab_frame(url: str, timeout: float = 10.0):
    """Return the first frame of the MJPEG stream at `url` as a PIL RGB Image."""
    from PIL import Image

    req = urllib.request.Request(url, headers={"User-Agent": "vision/grab"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        buf = b""
        while len(buf) < _MAX_BYTES:
            chunk = resp.read(8192)
            if not chunk:
                break
            buf += chunk
            start = buf.find(_SOI)
            if start == -1:
                continue
            end = buf.find(_EOI, start + 2)
            if end != -1:
                jpg = buf[start : end + 2]
                return Image.open(io.BytesIO(jpg)).convert("RGB")
    raise RuntimeError(f"no complete JPEG frame from {url}")
