"""Declarative tree DSL — the substrate humans and (later) an LLM compose with.

A node is a dict::

    {"type": "sequence"|"selector"|"parallel"|"<behavior-id>",
     "name":   optional str,
     "memory": optional bool          (composites; default True, selector False),
     "policy": "success_on_all"|"success_on_one"   (parallel only),
     "params": {...},                 (leaves: kwargs for the behaviour)
     "children": [ ...nodes... ]}     (composites)

``build_tree(spec, ctx)`` instantiates a py_trees subtree; ``tree_to_dsl(node)``
serialises a live tree back to this form so the current structure can be shown
or handed to an LLM. Round-trips leaf ids + params and composite structure.
"""

from __future__ import annotations

from typing import Any

import py_trees
from py_trees.common import ParallelPolicy

import conductor.behaviors  # noqa: F401  (populate the registry via @register)
from conductor import registry
from conductor.context import Context

_PARALLEL_POLICIES = {
    "success_on_all": lambda: ParallelPolicy.SuccessOnAll(synchronise=False),
    "success_on_one": lambda: ParallelPolicy.SuccessOnOne(),
}
_DEFAULT_MEMORY = {"sequence": True, "selector": False, "parallel": True}


def build_tree(spec: dict[str, Any], ctx: Context) -> py_trees.behaviour.Behaviour:
    """Instantiate a py_trees subtree from a DSL dict."""
    if not isinstance(spec, dict) or "type" not in spec:
        raise ValueError(f"tree node must be a dict with a 'type' key, got: {spec!r}")
    t = spec["type"]
    name = spec.get("name")

    if t in registry.COMPOSITES:
        children = [build_tree(c, ctx) for c in spec.get("children", [])]
        memory = bool(spec.get("memory", _DEFAULT_MEMORY[t]))
        if t == "sequence":
            return py_trees.composites.Sequence(name or "sequence", memory=memory, children=children)
        if t == "selector":
            return py_trees.composites.Selector(name or "selector", memory=memory, children=children)
        if t == "parallel":
            policy_key = spec.get("policy", "success_on_all")
            if policy_key not in _PARALLEL_POLICIES:
                raise ValueError(f"unknown parallel policy {policy_key!r}")
            return py_trees.composites.Parallel(
                name or "parallel", policy=_PARALLEL_POLICIES[policy_key](), children=children
            )

    entry = registry.get(t)
    params = spec.get("params", {})
    if not isinstance(params, dict):
        raise ValueError(f"'params' for {t!r} must be a dict, got: {params!r}")
    try:
        return entry.cls(ctx=ctx, name=name, **params)
    except TypeError as exc:
        raise ValueError(f"cannot build {t!r} with params {params!r}: {exc}") from exc


def tree_to_dsl(node: py_trees.behaviour.Behaviour) -> dict[str, Any]:
    """Serialise a live subtree back to DSL form (inverse of build_tree)."""
    if isinstance(node, py_trees.composites.Composite):
        if isinstance(node, py_trees.composites.Sequence):
            t = "sequence"
        elif isinstance(node, py_trees.composites.Selector):
            t = "selector"
        elif isinstance(node, py_trees.composites.Parallel):
            t = "parallel"
        else:  # pragma: no cover - other composites not used by this DSL
            t = "sequence"
        out: dict[str, Any] = {"type": t, "name": node.name}
        if t != "parallel":
            out["memory"] = getattr(node, "memory", _DEFAULT_MEMORY[t])
        out["children"] = [tree_to_dsl(c) for c in node.children]
        return out

    # leaf
    bt_id = getattr(node, "_bt_id", None)
    if bt_id is None:
        raise ValueError(f"leaf {node.name!r} ({type(node).__name__}) is not a @register'd behaviour")
    spec: dict[str, Any] = {"type": bt_id, "name": node.name}
    params = getattr(node, "dsl_params", {})
    if params:
        spec["params"] = dict(params)
    return spec
