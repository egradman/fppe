"""Behavior registry — the LLM-facing catalog.

Every leaf is tagged with ``@register(id, description, params=...)``. The
registry maps a stable string ``id`` to its class and metadata, and ``catalog()``
emits a JSON-serialisable list describing every behaviour and composite. That
catalog is what a human (``conductor catalog``) or, later, an LLM reads to know
what building blocks exist and how to parameterise them.

``params`` is a light JSON-schema-ish dict::

    {"value": {"type": "string", "enum": ["off", "gamepad", "pedals"],
               "description": "which base source drives the wheels+lift"}}
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class RegEntry:
    id: str
    cls: type
    description: str
    params: dict[str, Any] = field(default_factory=dict)


_REGISTRY: dict[str, RegEntry] = {}

#: The control-flow node types the DSL understands directly (not leaves).
COMPOSITES: dict[str, str] = {
    "sequence": "Run children in order; fail fast. Succeeds when all succeed.",
    "selector": "Try children in order; succeed on the first that succeeds (fallback).",
    "parallel": "Tick all children each cycle; policy decides success.",
}


def register(id: str, description: str, params: dict[str, Any] | None = None):
    """Class decorator that records a leaf behaviour under ``id``."""

    def deco(cls: type) -> type:
        if id in _REGISTRY:
            raise ValueError(f"behaviour id {id!r} already registered")
        cls._bt_id = id  # type: ignore[attr-defined]
        _REGISTRY[id] = RegEntry(id=id, cls=cls, description=description, params=params or {})
        return cls

    return deco


def get(id: str) -> RegEntry:
    try:
        return _REGISTRY[id]
    except KeyError:
        raise KeyError(
            f"unknown behaviour id {id!r}; known: {sorted(_REGISTRY)}"
        ) from None


def all_entries() -> list[RegEntry]:
    return [_REGISTRY[k] for k in sorted(_REGISTRY)]


def catalog() -> dict[str, Any]:
    """Full building-block catalog: leaves + composite control-flow nodes."""
    return {
        "composites": [
            {"type": t, "description": d, "children": "list of nodes"}
            for t, d in COMPOSITES.items()
        ],
        "behaviors": [
            {"type": e.id, "description": e.description, "params": e.params}
            for e in all_entries()
        ],
    }
