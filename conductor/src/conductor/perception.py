"""PerceptionClient — the camera-derived world input for behaviors.

Mirrors ``RobotClient``: a background thread grabs frames and publishes the
latest ``Sighting`` per active target, which behaviors read via ``get(target)``.
Detection is pluggable so this file carries no GPU/model dependency:

  * ``frame_source() -> frame``     — e.g. an MJPEG grabber off ``/mjpeg/cam0``.
  * ``detector(frame, target) -> Sighting | None`` — e.g. an open-vocab detector
    (OWL-ViT / Grounding-DINO) prompted with ``target``; ``None`` == not seen.

With no detector/frame_source wired, the client simply reports "not seen", so a
chase mission degrades to an endless search — safe and demonstrable before any
model is attached.

A short **grace window** debounces flickery detections: a target stays "seen"
for ``grace_s`` after its last positive frame, so ``Chase`` doesn't stutter.

``FakePerceptionClient`` lets tests script sightings directly.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

log = logging.getLogger(__name__)


@dataclass
class Sighting:
    seen: bool = False
    #: horizontal bearing to the target, radians, +left of image centre.
    bearing_rad: float = 0.0
    #: bbox area / frame area — a monotone proxy for closeness.
    area_frac: float = 0.0
    conf: float = 0.0
    #: monotonic time of the underlying detection.
    ts: float = 0.0


FrameSource = Callable[[], object]
Detector = Callable[[object, str], Optional[Sighting]]


class PerceptionClient:
    def __init__(
        self,
        detector: Detector | None = None,
        frame_source: FrameSource | None = None,
        poll_hz: float = 10.0,
        grace_s: float = 0.6,
    ) -> None:
        self._detector = detector
        self._frames = frame_source
        self._period = 1.0 / poll_hz
        self._grace = grace_s
        self._active: set[str] = set()
        self._last: dict[str, Sighting] = {}
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    # -- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None or self._detector is None or self._frames is None:
            return  # nothing to run without a real detector+source
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, name="perception", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def request(self, target: str) -> None:
        """Ask the detector to track ``target`` (called by StartPerception)."""
        with self._lock:
            self._active.add(target)

    def release(self, target: str) -> None:
        with self._lock:
            self._active.discard(target)

    # -- reads -----------------------------------------------------------
    def get(self, target: str) -> Sighting:
        with self._lock:
            s = self._last.get(target)
        if s is None:
            return Sighting()
        if s.seen and (time.monotonic() - s.ts) > self._grace:
            return replace(s, seen=False)  # grace expired
        return s

    # -- detection loop --------------------------------------------------
    def _loop(self) -> None:
        assert self._frames is not None and self._detector is not None
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            with self._lock:
                targets = list(self._active)
            if targets:
                try:
                    frame = self._frames()
                except Exception as exc:
                    log.debug("frame grab failed: %s", exc)
                    frame = None
                if frame is not None:
                    for target in targets:
                        try:
                            det = self._detector(frame, target)
                        except Exception as exc:
                            log.warning("detector failed for %r: %s", target, exc)
                            det = None
                        if det is not None:
                            with self._lock:
                                self._last[target] = replace(det, seen=True, ts=time.monotonic())
                        # miss: keep the last sighting; grace in get() expires it
            self._stop_evt.wait(max(0.0, self._period - (time.monotonic() - t0)))


class FakePerceptionClient:
    """Scriptable perception for tests. No thread, no model."""

    def __init__(self) -> None:
        self._s: dict[str, Sighting] = {}
        self.requested: list[str] = []
        self.released: list[str] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def request(self, target: str) -> None:
        self.requested.append(target)

    def release(self, target: str) -> None:
        self.released.append(target)

    def get(self, target: str) -> Sighting:
        return self._s.get(target, Sighting())

    # -- test controls ---------------------------------------------------
    def set_sighting(self, target: str, bearing_rad: float = 0.0, area_frac: float = 0.0,
                     conf: float = 0.9) -> None:
        self._s[target] = Sighting(seen=True, bearing_rad=bearing_rad, area_frac=area_frac,
                                   conf=conf, ts=time.monotonic())

    def clear(self, target: str) -> None:
        self._s[target] = Sighting()
