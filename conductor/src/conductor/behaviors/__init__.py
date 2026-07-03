"""Import every leaf module so their @register decorators populate the registry.

Importing ``conductor.behaviors`` (or anything under it) guarantees the full
catalog is available. ``conductor.safety`` registers a couple more internal
leaves; the DSL builder imports this package for its side effects.
"""

from conductor.behaviors import control, senders, conditions, chase  # noqa: F401

__all__ = ["control", "senders", "conditions", "chase"]
