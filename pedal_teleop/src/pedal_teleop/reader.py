"""Per-pedal USB-CDC reader thread.

Opens the serial port, parses 'ax,ay,az\\n' CSV lines from the XIAO
firmware, and exposes the latest sample (with timestamp) for the main
loop to consume. One thread per pedal — the firmware emits at 100 Hz
and we don't want one stalled serial port to block the other.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import serial


class PedalReader:
    def __init__(self, port: str, name: str, baud: int = 115200):
        self.port = port
        self.name = name
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._latest_ts: float = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"pedal-{name}")

    def start(self) -> None:
        # Discard any CDC warm-up garbage already buffered.
        self._ser.reset_input_buffer()
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass

    def latest(self) -> tuple[np.ndarray | None, float]:
        """Return (accel_g, monotonic_ts) of the most recent sample.

        accel_g is None until the first parseable line arrives.
        """
        with self._lock:
            return (None if self._latest is None else self._latest.copy(), self._latest_ts)

    def collect_rest(self, duration_s: float = 1.0) -> np.ndarray:
        """Block until `duration_s` of samples have arrived, return their mean."""
        deadline = time.monotonic() + duration_s
        samples: list[np.ndarray] = []
        last_seen_ts = 0.0
        while time.monotonic() < deadline:
            v, ts = self.latest()
            if v is not None and ts > last_seen_ts:
                samples.append(v)
                last_seen_ts = ts
            time.sleep(0.005)
        if not samples:
            raise RuntimeError(
                f"Pedal '{self.name}' produced no samples in {duration_s:.1f}s "
                f"on {self.port}. Is the firmware running?"
            )
        return np.mean(np.stack(samples, axis=0), axis=0)

    # ----------------------------------------------------------------- internals
    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        try:
            parts = line.strip().split(b",")
            if len(parts) != 3:
                return
            v = np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)
        except ValueError:
            return
        with self._lock:
            self._latest = v
            self._latest_ts = time.monotonic()
