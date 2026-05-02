# leader_teleop

Standalone uv-managed project for the **skynet** side of the leader/follower
teleop pipeline. Reads joint angles from two SO-101 leader arms over USB
serial and streams them as JSON-over-UDP packets to `viser_control.py` on
the **fppe** Pi.

This project is intentionally separate from the lerobot codebase: it has its
own `.venv`, three runtime deps (`feetech-servo-sdk`, `pydantic`, `pyserial`),
and talks to STS3215 servos directly via `scservo_sdk`.

## Hardware

The two leader arms enumerate as WCH CH343 USB-serial (`1a86:55d3`) on skynet:

| Side  | Stable path |
|-------|-------------|
| Left  | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056746-if00` |
| Right | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056664-if00` |

These are baked into `src/leader_teleop/calibrate.py`. If you swap arms,
update `LEFT_PORT` / `RIGHT_PORT` there.

## Setup

```bash
cd leader_teleop
uv sync          # creates .venv, ~12 MB total
```

## First-time calibration

Calibrations are stored at `~/.config/leader_teleop/{left,right}.json`.

```bash
just leader-calibrate                   # both arms
just leader-calibrate --arms left       # one arm only
```

You'll be prompted to:
1. Move the arm to the middle of its range and press ENTER (writes
   per-joint `Homing_Offset` so each motor's `Present_Position` is centred
   at 2048).
2. Sweep every joint through its full range, then press any key. The
   per-joint min/max ticks are recorded.

Override the calibration directory by setting `LEADER_TELEOP_CALIBRATION_DIR`.

## Streaming

Make sure `viser_control.py` is running on fppe with `--leader-udp-port 9999`
(this is the default), then on skynet:

```bash
just leader-teleop                       # both arms -> fppe at 100 Hz
just leader-teleop --arms left           # one arm only
just leader-teleop --rate 30             # slower
just leader-teleop --echo-only           # print packets locally instead of sending
```

The arms still need to be **locked** on the fppe side for the joint commands
to take effect — use the lock toggles in the web UI, or `POST /lock_arm`.
Streaming to an unlocked arm is a no-op; the follower's command handler
filters out joints whose side is unlocked.

## Wire format

JSON over UDP, defined in `src/leader_teleop/wire.py`. Both this project
and `examples/debug/viser_control.py` import the same module — viser_control
finds it via a `sys.path.insert` that points at `<repo>/leader_teleop/src`,
so no install is needed on the fppe side beyond having pydantic in its env.

New optional fields can be added without breaking older receivers
(Pydantic `model_config = ConfigDict(extra="ignore")`). Body joints are
in degrees (centred at the calibration midpoint); the gripper is 0..100
percent over its calibrated range.

## Layout

```
leader_teleop/
├── pyproject.toml          uv project config
├── .python-version         3.11
├── .venv/                  uv-managed (.gitignored)
└── src/leader_teleop/
    ├── arms.py             LeaderArm: scservo_sdk wrapper
    ├── calibration.py      Calibration dataclass + interactive flow
    ├── wire.py             Pydantic UDP packet schema
    ├── calibrate.py        CLI: leader-calibrate
    └── main.py             CLI: leader-teleop
```
