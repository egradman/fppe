"""Mode library — each ``*.yaml`` here is a mission DSL (the subtree that goes
into the engine's MissionSlot). ``load_mode`` / ``list_modes`` read them."""

from __future__ import annotations

from pathlib import Path

import yaml

_MODES_DIR = Path(__file__).resolve().parent


def list_modes() -> list[str]:
    return sorted(p.stem for p in _MODES_DIR.glob("*.yaml"))


def load_mode(name: str) -> dict:
    path = _MODES_DIR / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"unknown mode {name!r}; available: {list_modes()}")
    with path.open() as f:
        return yaml.safe_load(f)
