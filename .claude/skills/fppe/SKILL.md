---
name: fppe
description: Operate the LeKiwi robot end-to-end — stream leader-arm joints and foot-pedal base commands from skynet over UDP, and read/control follower state (positions, lock, lift, presets, e-stop, base driver selector) on the fppe Pi via viser_control.py's HTTP API.
---

# fppe robot

Two surfaces, one pipeline:

- **Skynet (leader side)** — `leader_teleop` reads two SO-101 leader arms over USB-serial and streams JSON-over-UDP packets at 100 Hz. `pedal_teleop` reads two foot-pedal IMUs over USB-CDC and streams body-velocity packets over UDP at 100 Hz.
- **fppe Pi (follower side)** — `examples/debug/viser_control.py` receives both UDP streams, drives the follower arms / base / lift, and exposes an HTTP API on `fppe:8091` for direct read/control. Start it with `just viser`.

Two gating switches:
- `teleop_mode` (`local`/`remote`) controls **arms**.
- `base_input_source` (`off`/`gamepad`/`pedals`) controls **wheels + lift**. Default is `off` — explicit-enable for safety.

For end-to-end teleop, both must be running and the target arms must be **locked** on the fppe side.

---

## Skynet — `leader_teleop`

uv-managed sub-project at `/home/egradman/dev/fppe/leader_teleop` (also `/home/dash/dev/fppe/leader_teleop`).

### Hardware / serial ports

Two WCH CH343 USB-serial adapters (`1a86:55d3`):

| Side  | Stable path                                                           |
|-------|-----------------------------------------------------------------------|
| Left  | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056746-if00`        |
| Right | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056664-if00`        |

Paths are baked into `src/leader_teleop/calibrate.py` as `LEFT_PORT` / `RIGHT_PORT`. Update there if you swap arms.

`/dev/ttyACM*` are `root:dialout 660` — the running user must be in the `dialout` group. `egradman` and `dash` both are. Group membership doesn't propagate to existing sessions; log out / back in (or `newgrp dialout`) if a fresh login is missing it.

### Calibrations

Stored at `~/.config/leader_teleop/{left,right}.json` for the running user (override with `LEADER_TELEOP_CALIBRATION_DIR`). Each user must calibrate once; calibrations are not shared between `egradman` and `dash`.

### Run via just (preferred)

From the repo root on skynet:

```bash
just leader-calibrate                    # calibrate both arms (interactive)
just leader-calibrate --arms left

just leader-teleop                       # stream both arms -> fppe at 100 Hz
just leader-teleop --arms right
just leader-teleop --rate 30
just leader-teleop --echo-only           # print packets locally, don't send
just leader-teleop --fppe-host 10.0.0.5  # override target host
```

These shell out to `cd leader_teleop && uv run …`.

### Run via uv directly

```bash
cd /home/egradman/dev/fppe/leader_teleop
uv sync                                  # first time only
uv run leader-calibrate
uv run leader-teleop
```

`uv sync` creates `.venv` in the project dir (~12 MB). Three runtime deps: `feetech-servo-sdk`, `pydantic`, `pyserial`. `uv` is installed system-wide at `/usr/local/bin/uv`.

### Running as a non-owner of the repo (e.g. `dash`)

The fppe checkout lives under `/home/egradman/dev/fppe` (with `/home/dash/dev/fppe` as a symlink to it). The repo and its in-tree `.venv` are owned by `egradman`, so a different user cannot write to them. To run cleanly, give the non-owner user its own uv cache **and** its own venv outside the repo:

```bash
# In dash's shell rc (or before `just`/`uv` invocations)
export UV_CACHE_DIR=$HOME/.cache/uv
export UV_PROJECT_ENVIRONMENT=$HOME/.venvs/leader_teleop
```

Then `uv sync` / `uv run` from inside `~/dev/fppe/leader_teleop` writes the venv to `~/.venvs/leader_teleop` and caches wheels under `~/.cache/uv` — both fully under the running user's home. Don't share `UV_CACHE_DIR` across users; uv's cache is not designed to be multi-user-writable and you'll get permission errors. Each user also calibrates separately (see above).

### Calibration flow (interactive)

1. Move the arm to the middle of its range; press ENTER. Per-joint `Homing_Offset` is written so each motor's `Present_Position` is centred at 2048.
2. Sweep every joint through its full range; press any key. Per-joint min/max ticks are recorded.

If a calibration already exists you'll be asked before overwriting.

### Streaming preconditions

- `viser_control.py` must be running on the fppe Pi listening on UDP `9999` (its default). Start with `just viser`, or rely on labwc autostart on the Pi.
- Each arm must be **locked** on the fppe side for joint commands to take effect. Streaming to an unlocked arm is a no-op (the follower filters out joints whose side is unlocked).

Default UDP target is `fppe:9999`. Override with `--fppe-host` / `--port`.

### Wire format

JSON over UDP, schema in `leader_teleop/src/leader_teleop/wire.py`. The fppe-side `viser_control.py` imports the same module via a `sys.path.insert` — no install needed on the Pi beyond pydantic. Body joints are degrees (centred at the calibration midpoint); gripper is 0–100 % over its calibrated range. Schema accepts unknown fields (`extra="ignore"`), so adding optional fields is backward-compatible.

### Stop / kill

`just leader-teleop` is foreground; Ctrl-C cleanly disconnects. Force-kill a stray instance:

```bash
pkill -f leader_teleop
```

---

## Skynet — `pedal_teleop`

uv-managed sub-project at `/home/egradman/dev/fppe/pedal_teleop`. Reads two Seeed XIAO nRF52840 Sense boards (each carrying an LSM6DS3TR-C IMU) over USB-CDC, projects gravity onto a per-pedal user frame, combines into body velocities + lift commands, and streams over UDP to `fppe:9998`.

### Hardware / serial ports

Two XIAOs, identified by USB serial number burned in by the bootloader. Hardcoded in `src/pedal_teleop/ports.py`:

| Side  | Stable path                                                                       |
|-------|-----------------------------------------------------------------------------------|
| Left  | `/dev/serial/by-id/usb-Arduino_Seeed_XIAO_nRF52840_Sense_423226579BF5DBF7-if00`   |
| Right | `/dev/serial/by-id/usb-Arduino_Seeed_XIAO_nRF52840_Sense_E8D9BA0B04428993-if00`   |

To re-identify after a hardware swap: unplug one pedal, run `ls /dev/serial/by-id/`, see which serial is still present — that's the one still plugged in. Label the pedal accordingly and update `ports.py`.

### Firmware

Arduino sketch at `firmware/pedal_imu/pedal_imu.ino`. Emits raw accel as `ax,ay,az\n` ASCII CSV over USB-CDC at 100 Hz. Identical .uf2 on both pedals. Flash by double-tapping RESET on the XIAO and dragging the compiled .uf2 onto the mounted XIAO-BOOT volume.

### Mounting convention

The XIAO is assumed mounted on each pedal with the USB connector pointing AWAY from the user and the top face (components up) facing the sky. With that orientation:

- Body +X = forward (toe-down direction)
- Body +Y = right (roll-right direction)

If a pedal is mounted rotated, flip the matching `AXIS_SIGN_FORWARD_*` / `AXIS_SIGN_RIGHT_*` constant in `ports.py` (no firmware reflash needed).

### Startup behaviour (auto-zero + sanity check)

On start, `pedal_teleop` averages ~1 s of accel per pedal as the "rest gravity" vector and verifies its magnitude falls inside `[0.95g, 1.05g]`. If anything is resting on a pedal (foot, tool, hand) the magnitude check fails and the process exits with a clear error — restart with feet off.

### Mapping

Each pedal acts like a 2-axis analog stick (tilt-forward = pitch, tilt-right = roll):

```
x_vel (m/s, +forward)        = LIN_SPEED * (pitch_L + pitch_R) / 2
y_vel (m/s, +strafe-left)    = -LIN_SPEED * (roll_L + roll_R) / 2
theta (deg/s, +CCW / +left)  = ANG_SPEED * (pitch_R - pitch_L) / 2
lift  = "up"   if roll_L < -DZ AND roll_R > +DZ  (both outward)
      = "down" if roll_L > +DZ AND roll_R < -DZ  (both inward)
      = "stop" otherwise
```

Sign conventions match `body_to_wheel_raw` in `viser_control.py` so pedal output behaves identically to gamepad output of the same magnitude.

### Run via just

```bash
just pedal-teleop                                # stream pedals -> fppe at 100 Hz
just pedal-teleop --echo-only                    # print JSON packets locally
just pedal-teleop --visualize                    # ncurses live view of raw axes + output cmds
just pedal-teleop --fppe-host 10.0.0.5           # override target host
```

### Selecting pedals on the fppe side

Pedal packets are decoded but **not acted on** unless `base_input_source == "pedals"` on the fppe side. Toggle from the web UI's top bar (3-way: Off / Gamepad / Pedals) or via HTTP:

```bash
curl -s -X POST http://fppe:8091/base_input_source \
  -H 'Content-Type: application/json' -d '{"value": "pedals"}'
```

Default is `off`. Any change immediately stops the wheels and lift.

### Wire format

JSON over UDP on port `9998`, schema in `pedal_teleop/src/pedal_teleop/wire.py`. Backwards-compatible (`extra="ignore"`); add new optional fields freely.

---

## fppe Pi — viser_control.py HTTP API

`examples/debug/viser_control.py` exposes an HTTP API on `fppe:8091` for reading and writing follower-robot state. Start it with `just viser` (or it autostarts via labwc on the Pi).

### Motor names

Left arm (IDs 21-26): `arm_left_shoulder_pan`, `arm_left_shoulder_lift`, `arm_left_elbow_flex`, `arm_left_wrist_flex`, `arm_left_wrist_roll`, `arm_left_gripper`

Right arm (IDs 1-6): `arm_right_shoulder_pan`, `arm_right_shoulder_lift`, `arm_right_elbow_flex`, `arm_right_wrist_flex`, `arm_right_wrist_roll`, `arm_right_gripper`

All positions are raw servo values 0–4095. Midpoint is 2048.

Gripper direction: lower values = closed, higher values = open. Home position (~1556–1611) is open; ~1000 is closed.

### Read state

```bash
curl -s http://fppe:8091/state
```

Returns JSON:
```json
{
  "positions": {"arm_left_shoulder_pan": 2045, ...},
  "arm_locked": {"left": true, "right": true}
}
```

### Set joint positions

Arms must be locked (torque enabled) for position commands to take effect.

```bash
# Single joint
curl -s -X POST http://fppe:8091/goal_position \
  -H 'Content-Type: application/json' \
  -d '{"arm_right_elbow_flex": 2048}'

# Multiple joints
curl -s -X POST http://fppe:8091/goal_position \
  -H 'Content-Type: application/json' \
  -d '{"arm_right_shoulder_lift": 1920, "arm_right_elbow_flex": 1472}'
```

### Lock / unlock arms

Locked = torque enabled (arm holds position, sliders / leader stream control it). Unlocked = torque disabled (arm goes limp, sliders show feedback only, leader stream is ignored for that side).

```bash
curl -s -X POST http://fppe:8091/lock_arm \
  -H 'Content-Type: application/json' \
  -d '{"side": "right", "lock": true}'

curl -s -X POST http://fppe:8091/lock_arm \
  -H 'Content-Type: application/json' \
  -d '{"side": "left", "lock": false}'
```

### Preset positions

```bash
curl -s -X POST http://fppe:8091/go_home       # arms folded down
curl -s -X POST http://fppe:8091/go_arms_up
curl -s -X POST http://fppe:8091/go_middle     # all joints to 2048
```

These automatically lock both arms before moving.

### Elevator control

```bash
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "up"}'
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "down"}'
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "stop"}'
```

The lift runs at fixed velocity until stopped.

### Emergency stop

```bash
curl -s -X POST http://fppe:8091/estop
```

Disables torque on all motors immediately. Both arms go limp, lift stops.

---

## Quick sanity checks

```bash
# Skynet side
ls -l /dev/serial/by-id/                          # both leader adapters enumerated
groups                                            # must include 'dialout'
cat ~/.config/leader_teleop/left.json             # calibrations exist

# fppe side
curl -s http://fppe:8091/state | jq .arm_locked   # arms are locked
nc -u -z fppe 9999 && echo ok                     # UDP listener (best-effort)
```
