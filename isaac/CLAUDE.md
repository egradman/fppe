# Isaac Sim / Isaac Lab — fppe sim workspace

This directory holds everything related to simulating the fppe robot (LeKiwi base + lift + dual SO-101 arms) in Isaac Sim, driving it from the existing fppe teleop stack, and eventually training RL policies on it.

The goal is sim-to-real for the same hardware controlled by `examples/debug/viser_control.py`.

## Host environment (skynet)

| Thing | Value |
|---|---|
| GPU | NVIDIA RTX 5080 (Blackwell, 16 GB) |
| Driver | 580.159.03 / CUDA 13.0 |
| OS | Ubuntu 24.04 (kernel 6.17) |
| Display | X server, Sunshine/Moonlight streamable (only when egradman owns the X session) |

Blackwell support landed in Isaac Sim 5.0; the 5.1 install here is the first stable line that works on a 5080.

## Isaac install layout

All Isaac code lives outside this repo, under `~/dev/isaac/`:

```
~/dev/isaac/
├── env_isaaclab/        # Python 3.11 venv — activate this for any Isaac work
└── IsaacLab/            # git clone of github.com/isaac-sim/IsaacLab (on `main`)
    ├── isaaclab.sh      # locally modified (uncommitted change)
    └── source/
        ├── isaaclab/            -> pip-installed editable as `isaaclab` 0.54.3
        ├── isaaclab_assets/     -> `isaaclab_assets` 0.2.4
        ├── isaaclab_tasks/      -> `isaaclab_tasks` 0.11.16
        ├── isaaclab_rl/         -> `isaaclab_rl` 0.5.1
        ├── isaaclab_mimic/      -> `isaaclab_mimic` 1.0.16
        └── isaaclab_contrib/    -> `isaaclab_contrib` 0.0.2
```

Pip-installed (into `env_isaaclab`):

- `isaacsim` 5.1.0.0 (and all `isaacsim-*` subpackages — kernel, gui, replicator, etc.)
- `torch` 2.7.0+cu128
- `warp-lang` 1.12.1

Flavor: **pip-into-venv**, not the old Omniverse Launcher route. The Launcher leftovers at `~/Documents/Kit/apps/` and `~/.nvidia-omniverse/` are stale — ignore them.

Note: the `VERSION` file at the IsaacLab root says `2.3.2`, but the installed `isaaclab` package reports `0.54.3` because main has moved on. Trust the pip version.

## Activating the env

```bash
source ~/dev/isaac/env_isaaclab/bin/activate
# now `python` is /home/egradman/dev/isaac/env_isaaclab/bin/python3.11
# `import isaacsim, isaaclab` works
```

`isaaclab.sh` (wrapper script) lives at `~/dev/isaac/IsaacLab/isaaclab.sh` — use it for running headless training jobs and the standalone demos.

## Smoke-test status

Last verified 2026-05-20:

- `import isaacsim` ✓
- `import isaaclab` ✓
- Has not been actually launched / rendered this session — that's still TODO.

If the GUI fails to come up after an OS/driver update, the usual suspects are: stale USD cache (`~/.cache/ov/` or `~/.nvidia-omniverse/`), Vulkan/X mismatch, or Python wheel ABI drift after a driver bump.

## fppe robot — what the sim needs to model

From the fppe codebase (`examples/debug/viser_control.py`, `examples/debug/motors.py`, `examples/debug/wheels.py`):

- **Base**: 3-wheel omni drive (LeKiwi-style)
- **Lift**: single prismatic joint between base and arm mounting plate
- **Arms**: two 6DOF SO-101 arms (right IDs 1–6, left IDs 21–26 on a single Feetech bus)
- **Control**: position targets per joint; body-frame velocity for the base

### Existing URDF in-tree

**`examples/debug/assets/alohamini/Aloha.urdf`** — full robot description, 1065 lines, 19 links, 18 joints. All STL meshes alongside in `meshes/`.

Joint inventory matches the hardware:

| Count | Type | What |
|---|---|---|
| 3 | continuous | omni wheels (`wheel1`, `wheel2`, `wheel3`) |
| 1 | prismatic (limited) | `vertical_link` = the lift |
| 6 | revolute (limited) | right SO-101 (`right_link1`..`right_link6`) |
| 6 | revolute (limited) | left SO-101 (`left_link1`..`left_link6`) |
| 2 | fixed | left/right arm base mounts to the lift |

That's 16 actuated DoF. Joint names already use the `right_` / `left_` prefix convention.

This skips most of M1's pain — no URDF assembly needed, just URDF → USD conversion via Isaac's importer.

## Teleop stack to wire into sim

The same UDP / HTTP surfaces that drive the real Pi today:

- `leader_teleop/` — streams leader-arm joint angles over UDP from skynet
- `pedal_teleop/` — foot pedals for base velocity
- `examples/debug/gamepad_teleop.py` — gamepad for base + lift
- `examples/debug/viser_control.py` — HTTP API on the Pi consumes all of the above

**Plan**: build a sim-side consumer of the same UDP packets so the existing teleop senders can drive sim or real with a single flag. Don't fork teleop; redirect it.

## Re-converting the URDF

Whenever `Aloha.urdf` changes, regenerate the USD:

```bash
source ~/dev/isaac/env_isaaclab/bin/activate
rm -rf isaac/assets/aloha.usd isaac/assets/configuration isaac/assets/.asset_hash isaac/assets/config.yaml
python ~/dev/isaac/IsaacLab/scripts/tools/convert_urdf.py \
  examples/debug/assets/alohamini/Aloha.urdf \
  isaac/assets/aloha.usd \
  --joint-target-type position \
  --headless
```

The `rm -rf` is needed because the converter caches by hash and silently no-ops if outputs exist. The "Joint Axis is not body aligned" warnings during conversion are normal — PhysX auto-aligns.

## Current joint-limit values (placeholders — refine later)

Written into `Aloha.urdf` for all revolute and prismatic joints; previously they were all zero (SolidWorks exporter default).

- **All 12 arm revolutes**: `lower=-π, upper=π, effort=10 Nm, velocity=10 rad/s` — generous Feetech STS3215-ish envelope, not tuned per-joint to real SO-101 limits.
- **`vertical_move` (lift)**: `lower=0, upper=0.5 m, effort=200 N, velocity=0.5 m/s` — the 0.5 m upper is a **guess**; need to measure actual mechanical travel.
- Wheels are continuous, no `<limit>` element.

TODO: replace arm limits with the real SO-101 per-joint ranges from LeRobot upstream, and measure the lift travel.

## Roadmap (where we are: M1 done, M2 next)

- **M0** — Install Isaac Sim 5.1 + Isaac Lab on skynet. ✓ Done (pre-existing).
- **M1** — Convert `examples/debug/assets/alohamini/Aloha.urdf` to USD, load in Isaac, verify all 16 actuated joints move under position control. ✓ Done 2026-05-20. Converter: `~/dev/isaac/IsaacLab/scripts/tools/convert_urdf.py`. Verify script: `isaac/m1_verify.py`. **Gotcha:** the SolidWorks-exported URDF had every `<limit>` block zeroed (`lower=0 upper=0 effort=0 velocity=0`); had to write real values before conversion. A `.orig` backup is at `Aloha.urdf.orig`.
- **M2** — Sim-side UDP consumer for leader_teleop / pedal_teleop; drive sim robot from skynet's existing senders.
- **M3** — Record sim demos as LeRobotDataset (parquet + videos). Sim observations must match real 1:1 (camera res/FPS/joint order).
- **M4** — First RL task: single arm reaches a randomized xyz target. PPO via `rsl_rl`, manager-based env, ~4096 parallel envs.

Don't skip ahead. Each milestone validates the layer below it.

## Conventions for code in this directory

- All scripts assume `source ~/dev/isaac/env_isaaclab/bin/activate` first.
- Sim-side teleop consumers should reuse the wire format already in `leader_teleop/` and `pedal_teleop/` — no schema fork.
- Joint naming: prefix `right_` / `left_` for the two arms so URDF joint names don't collide. Match the motor-ID ordering already in `examples/debug/motors.py`.
- USDs and large binary assets do **not** go in git. Put them under `isaac/assets/` and add to `.gitignore`. URDFs and Python sources do go in git.
