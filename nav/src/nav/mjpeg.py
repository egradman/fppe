"""Threaded reader for viser_control's MJPEG camera feeds.

viser_control serves `multipart/x-mixed-replace` JPEG streams at
`http://<host>:8091/mjpeg/<label>` (the fisheye nav cam is `cam2`). We parse the
multipart stream by hand off urllib — DIY MJPEG servers trip up ffmpeg's HTTP
demuxer, and this keeps the dependency surface to the stdlib + Pillow.

The reader keeps the single latest decoded frame plus a rolling deque of the
last N frames (ViNT's context window), so the executor can grab a coherent
`(context, latest)` snapshot each control tick without blocking on the network.
"""

import threading
import time
from collections import deque
from io import BytesIO
from urllib.request import urlopen

from PIL import Image as PILImage

_SOI = b"\xff\xd8"  # JPEG start-of-image
_EOI = b"\xff\xd9"  # JPEG end-of-image


class MjpegReader:
    def __init__(self, url: str, context_len: int = 6, timeout: float = 5.0):
        self.url = url
        self.timeout = timeout
        self._lock = threading.Lock()
        self._latest: PILImage.Image | None = None
        self._context: deque[PILImage.Image] = deque(maxlen=context_len)
        self._frame_count = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mjpeg")

    def start(self) -> "MjpegReader":
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def snapshot(self) -> tuple[list[PILImage.Image], PILImage.Image] | None:
        """Return (context_frames_oldest_to_newest, latest) or None if not ready.

        `context_frames` has exactly `context_len` entries once the buffer has
        filled; before then, returns None so the caller waits for warmup.
        """
        with self._lock:
            if self._latest is None or len(self._context) < self._context.maxlen:
                return None
            return list(self._context), self._latest

    def _publish(self, jpeg: bytes):
        try:
            img = PILImage.open(BytesIO(jpeg)).convert("RGB")
        except Exception:
            return  # partial/corrupt frame; skip
        with self._lock:
            self._latest = img
            self._context.append(img)
            self._frame_count += 1

    def _run(self):
        while not self._stop.is_set():
            try:
                stream = urlopen(self.url, timeout=self.timeout)
                buf = b""
                while not self._stop.is_set():
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    # Extract every complete JPEG currently in the buffer.
                    while True:
                        start = buf.find(_SOI)
                        if start < 0:
                            break
                        end = buf.find(_EOI, start + 2)
                        if end < 0:
                            buf = buf[start:]  # keep partial frame
                            break
                        self._publish(buf[start:end + 2])
                        buf = buf[end + 2:]
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[nav] MJPEG reader ({self.url}) error: {e}; retrying...")
                    time.sleep(1.0)
