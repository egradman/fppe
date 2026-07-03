"""SenderManager — lifecycle of the skynet-side UDP sender processes.

The command *sources* for the robot live on skynet as separate uv projects and
stream to the fppe Pi over UDP (see the justfile and each project's wire.py):

  leader  ->  `uv run leader-teleop`   (arms  -> fppe:9999)
  pedal   ->  `uv run pedal-teleop`    (base  -> fppe:9998)
  nav     ->  (future) visual-nav sender (base -> fppe:nav)

Making the tree own these processes is what lets a "mode" be self-contained:
entering teleop launches leader + pedal; leaving it kills them. Kill-on-exit is
guaranteed via an ``atexit`` hook so a conductor crash cannot leave a
base-driving sender alive (the Pi-side e-stop and ``base_input_source=off`` are
the further backstops).

``FakeSenderManager`` tracks a running-set with no real processes, for tests.
"""

from __future__ import annotations

import atexit
import logging
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class SenderSpec:
    #: subdirectory under the repo root that holds the uv project.
    cwd: str
    #: argv to launch it (relative to that cwd).
    cmd: tuple[str, ...]
    #: whether this sender is wired up yet. nav is not (Phase-3 seam).
    available: bool = True
    #: if the sender accepts ``--fppe-host``, forward the configured host.
    accepts_host: bool = True


SENDERS: dict[str, SenderSpec] = {
    "leader": SenderSpec(cwd="leader_teleop", cmd=("uv", "run", "leader-teleop")),
    "pedal": SenderSpec(cwd="pedal_teleop", cmd=("uv", "run", "pedal-teleop")),
    # Phase-3 seam: no "nav" base_input_source or nav UDP sender exists yet.
    "nav": SenderSpec(cwd="nav", cmd=("uv", "run", "nav-sender"), available=False),
}


def repo_root() -> Path:
    """Repo root: conductor/src/conductor/senders.py -> .../fppe."""
    return Path(__file__).resolve().parents[3]


class SenderManager:
    """Spawns/kills real sender subprocesses."""

    def __init__(self, root: Path | None = None, fppe_host: str = "fppe") -> None:
        self._root = root or repo_root()
        self._fppe_host = fppe_host
        self._procs: dict[str, subprocess.Popen] = {}
        self._lock = threading.Lock()
        atexit.register(self.stop_all)

    def start(self, name: str) -> None:
        spec = SENDERS.get(name)
        if spec is None:
            raise ValueError(f"unknown sender {name!r}; known: {list(SENDERS)}")
        if not spec.available:
            raise RuntimeError(
                f"sender {name!r} is not wired up yet (Phase-3 nav integration)"
            )
        with self._lock:
            if self.is_running(name):
                return
            argv = list(spec.cmd)
            if spec.accepts_host:
                argv += ["--fppe-host", self._fppe_host]
            cwd = self._root / spec.cwd
            log.info("starting sender %s: %s (cwd=%s)", name, " ".join(argv), cwd)
            self._procs[name] = subprocess.Popen(argv, cwd=str(cwd))

    def stop(self, name: str) -> None:
        with self._lock:
            proc = self._procs.pop(name, None)
        if proc is None:
            return
        log.info("stopping sender %s (pid=%s)", name, proc.pid)
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            log.warning("sender %s did not exit; killing", name)
            proc.kill()

    def is_running(self, name: str) -> bool:
        proc = self._procs.get(name)
        return proc is not None and proc.poll() is None

    def running(self) -> list[str]:
        return [n for n in list(self._procs) if self.is_running(n)]

    def stop_all(self) -> None:
        for name in list(self._procs):
            self.stop(name)


class FakeSenderManager:
    """Tracks a running-set; no real processes. Tests only."""

    def __init__(self, fail_unavailable: bool = True) -> None:
        self._running: set[str] = set()
        self._fail_unavailable = fail_unavailable
        self.calls: list[tuple] = []

    def start(self, name: str) -> None:
        self.calls.append(("start", name))
        spec = SENDERS.get(name)
        if spec is None:
            raise ValueError(f"unknown sender {name!r}")
        if self._fail_unavailable and not spec.available:
            raise RuntimeError(f"sender {name!r} not wired up yet")
        self._running.add(name)

    def stop(self, name: str) -> None:
        self.calls.append(("stop", name))
        self._running.discard(name)

    def is_running(self, name: str) -> bool:
        return name in self._running

    def running(self) -> list[str]:
        return sorted(self._running)

    def stop_all(self) -> None:
        for name in list(self._running):
            self.stop(name)
