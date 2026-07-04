# nav — visual navigation for the LeKiwi base (skynet side)

Language-instruction navigation over a pre-recorded map, built on the open
GNM/ViNT/NoMaD policies (`robodhruv/visualnav-transformer`, MIT). LM-Nav-style:
a topological graph of camera frames + CLIP landmark grounding + an LLM to parse
instructions. Inference runs here on skynet's GPU; body-velocity commands stream
to the fppe Pi over UDP (a future `base_input_source == "nav"`), mirroring
`leader_teleop` / `pedal_teleop`.

See `memory/project_nav_stack.md` for the full plan and rationale, and `PLAN.md`
for Phases 2→6.

## Status: Phase 2 — the executor (drive to one goal photo)

Phase 1 (offline harness) is done. Phase 2 closes the loop on real hardware:
read the live fisheye feed → ViNT → proportional controller → UDP → the fppe
base. No graph/labels/language yet — a single hop toward a goal photo.

- **fppe side (`viser_control.py`):** `base_input_source` gained a **`nav`** value
  and a nav UDP listener on **:9997** (same `PedalPacket` wire as pedals, own
  port, own gate). Enable with the web UI's base-source control.
- **skynet side (this project):** `nav-drive` runs the control loop; `nav-snap`
  grabs a goal photo off the fisheye.

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

## Run the executor (Phase 2)

`viser_control.py` must be up on fppe (the nav listener is on by default). Then,
from skynet:

```bash
# 1. capture a goal photo a few meters ahead (safe; no base movement)
uv run nav-snap --out goal.jpg

# 2. drive toward it. Set base_input_source='nav' in the web UI first,
#    and KEEP A HAND ON THE PHYSICAL E-STOP for the first run.
uv run nav-drive --goal goal.jpg --robot-host fppe
```

Starts slow (`--speed 0.12` m/s). Stops when ViNT's distance-to-goal drops below
`--goal-dist`. If the base strafes or rotates the wrong way on the first run,
flip `--invert-y` / `--invert-theta`. `--arm` auto-sets the base source over HTTP
(off by default — arm manually until you trust it). Ctrl-C stops; the 300 ms
wheel watchdog is the backstop if the stream ever stalls.

## Layout

- `configs/vint.yaml` — trimmed model config (the keys `get_model` reads).
- `src/nav/preprocess.py` — ROS-free `transform_images` (matches training).
- `src/nav/model.py` — `load_config` / `load_model` via the vendored factory.
- `src/nav/harness.py` — Phase 1 CLI + `run_vint` (reused by the executor).
- `src/nav/mjpeg.py` — threaded reader for viser_control's MJPEG feeds.
- `src/nav/controller.py` — ViNT waypoints → body velocity `(vx, vy, ω)`.
- `src/nav/wire.py` — `PedalPacket`-compatible UDP encoder (:9997).
- `src/nav/executor.py` — Phase 2 control loop (`nav-drive`) + `nav-snap`.
