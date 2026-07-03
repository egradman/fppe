"""Context — the shared handles every behaviour needs.

The DSL builder injects one ``Context`` into every leaf so behaviors can issue
commands (``client``), manage sources (``senders``), drive the base autonomously
(``base``), and read the camera-derived world (``perception``) without any
globals. State *reads* go through ``client.snapshot()`` / ``perception.get()``.

``base`` and ``perception`` default to fakes, so modes that don't use them
(idle, teleop) construct a Context exactly as before.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from conductor.base_cmd import BaseCmdChannel, FakeBaseCmdChannel
from conductor.client import RobotClientBase
from conductor.perception import FakePerceptionClient, PerceptionClient
from conductor.senders import FakeSenderManager, SenderManager


@dataclass
class Context:
    client: RobotClientBase
    senders: "SenderManager | FakeSenderManager"
    fppe_host: str = "fppe"
    base: "BaseCmdChannel | FakeBaseCmdChannel" = field(default_factory=FakeBaseCmdChannel)
    perception: "PerceptionClient | FakePerceptionClient" = field(default_factory=FakePerceptionClient)
