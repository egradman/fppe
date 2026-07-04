# nav — visual navigation for the LeKiwi base (skynet side)

Language-instruction navigation over a pre-recorded map, built on the open
GNM/ViNT/NoMaD policies (`robodhruv/visualnav-transformer`, MIT). LM-Nav-style:
a topological graph of camera frames + CLIP landmark grounding + an LLM to parse
instructions. Inference runs here on skynet's GPU; body-velocity commands stream
to the fppe Pi over UDP (a future `base_input_source == "nav"`), mirroring
`leader_teleop` / `pedal_teleop`.

See `memory/project_nav_stack.md` for the full plan and rationale.

## Status: Phase 1 — offline inference harness

Proves the policy loads on this GPU and emits sane `(distance, waypoint)`
tensors. Does **not** touch the robot.

## Setup

Requires the RTX 5080 (Blackwell) → cu128 torch wheels (handled by the
`pytorch-cu128` index in `pyproject.toml`).

```bash
cd nav
uv sync                     # installs torch (cu128) + model deps
```

Model code under `src/nav/models/` is **vendored** from the official
visualnav-transformer repo (Berkeley, MIT) — no third-party package. Weights are
the official pretrained checkpoints, auto-downloaded on first run from the
Pre-Trained Models Google Drive folder into `nav/weights/` (gitignored).

## Run the harness

```bash
uv run nav-harness --media /path/to/some_driving_or_indoor.mp4
```

First run downloads the ViNT checkpoint (~gdown) into the platformdirs cache.
Expected output: model param count, `obs_img`/`goal_img` shapes, a scalar
temporal distance, and 5 predicted waypoints.

## Layout

- `configs/vint.yaml` — trimmed model config (the keys `get_model` reads).
- `src/nav/preprocess.py` — ROS-free `transform_images` (matches training).
- `src/nav/model.py` — `load_config` / `load_model` via general_navigation factory.
- `src/nav/harness.py` — Phase 1 CLI.
