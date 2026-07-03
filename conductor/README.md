# conductor — behavior-tree modes for the fppe robot

A [py_trees](https://py-trees.readthedocs.io) "conductor" that runs on **skynet**,
acts as a client of the fppe Pi's `viser_control` HTTP API (`:8091`), and owns the
lifecycle of the skynet-side UDP sender processes. Robot **modes** (idle, teleop,
nav, clean-room) are subtrees; switching a mode swaps a subtree; the GPIO e-stop is
a top-level preemption guard that gives clean teardown for free.

This project sits *alongside* the existing web UI and manual flow — it's purely
additive. Nothing in `viser_control.py` changes.

## Why a behavior tree

Today a "mode" is a human flipping two enums on the Pi (`teleop_mode`,
`base_input_source`) and launching/killing the right sender by hand. The conductor
makes modes **declarative and composable**, so new behaviors can be authored, tied
together into sequenceable activities, and — ultimately — snapped together and
edited at runtime by an LLM.

## Run

```bash
just conductor list            # available modes
just conductor show teleop     # render a mode's tree (no hardware)
just conductor catalog         # the LLM-facing behavior catalog (JSON)
just conductor run teleop      # tick the engine against a live fppe Pi
```

`run` expects `viser_control.py` up on the Pi (`just viser`). Ctrl-C returns to
`idle` and stops all senders.

## Layout

- `src/conductor/client.py` — `RobotClient`: typed wrapper over `:8091` + a
  background `/state` poller. `FakeRobotClient` backs the offline tests.
- `src/conductor/senders.py` — `SenderManager`: spawns/kills `leader-teleop` /
  `pedal-teleop` (and future `nav`); guaranteed kill-on-exit.
- `src/conductor/behaviors/` — the leaf library. Each `@register`'d behaviour is a
  thin, non-blocking py_trees node over the client/senders.
- `src/conductor/registry.py` — `@register` + `catalog()` (the LLM tool-catalog).
- `src/conductor/dsl.py` — `build_tree(dsl)` ⇄ `tree_to_dsl(node)`; the JSON/YAML
  tree DSL humans and LLMs compose with.
- `src/conductor/safety.py` — the `Selector[safety, mission]` root and `SafeStop`.
- `src/conductor/engine.py` — the ticking runtime + control surface
  (`load_mission` / `append` / `interrupt` / `abort`).
- `src/conductor/modes/*.yaml` — mission DSL docs.

## The two invariants worth knowing

1. **Non-blocking ticks.** Behaviors never block: side effects fire in
   `initialise()`, and `update()` returns `RUNNING` until the polled snapshot
   confirms the effect. The engine ticks at ~10 Hz.
2. **Teardown keys on `INVALID`.** `StartSender` kills its process only when the
   node is stopped with `INVALID` (preemption/swap), never on `SUCCESS`. py_trees
   propagates `stop(INVALID)` through a preempted subtree, so mode switches and
   e-stops tear resources down automatically. This is the whole reason a mode can
   be "self-contained".

## Status & roadmap

**Real:** `idle`, `teleop`. **Stub (wired, honest FAILURE):** `nav`, `clean_room`.

- **Phase 3 — nav:** add a `"nav"` value to `viser_control` `BaseInputSource._VALUES`
  + a `start_nav_udp_listener` mirroring `start_pedal_udp_listener`, and a nav sender
  in `nav/` emitting `PedalPacket`-shaped body velocities. Then `navigate_to` becomes
  real.
- **Phase 4 — LLM:** an Anthropic tool-use loop with tools `list_behaviors`
  (→ `catalog()`), `get_tree` (→ `tree_to_dsl`), and `set_mission`/`interrupt`/
  `append`/`abort` (→ the Engine API). The registry + DSL round-trip are the substrate.

## Test

```bash
uv run pytest        # offline; FakeRobotClient + FakeSenderManager, no hardware
```
