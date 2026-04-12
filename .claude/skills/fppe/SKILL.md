---
name: fppe
description: Read and control the LeKiwi robot's arm positions, lock state, and elevator via the HTTP API served by viser_control.py on the fppe host.
---

# Robot Remote Control

The viser control server (`examples/debug/viser_control.py`) exposes an HTTP API on `fppe:8091` for reading and writing robot state. The server must be running first (`just viser`).

## Motor names

Left arm (IDs 21-26): `arm_left_shoulder_pan`, `arm_left_shoulder_lift`, `arm_left_elbow_flex`, `arm_left_wrist_flex`, `arm_left_wrist_roll`, `arm_left_gripper`

Right arm (IDs 1-6): `arm_right_shoulder_pan`, `arm_right_shoulder_lift`, `arm_right_elbow_flex`, `arm_right_wrist_flex`, `arm_right_wrist_roll`, `arm_right_gripper`

All positions are raw servo values 0-4095. Midpoint is 2048.

Gripper direction: lower values = closed, higher values = open. Home position (~1556-1611) is open; ~1000 is closed.

## Read state

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

## Set joint positions

Arms must be locked (torque enabled) for position commands to take effect.

```bash
# Move a single joint
curl -s -X POST http://fppe:8091/goal_position \
  -H 'Content-Type: application/json' \
  -d '{"arm_right_elbow_flex": 2048}'

# Move multiple joints at once
curl -s -X POST http://fppe:8091/goal_position \
  -H 'Content-Type: application/json' \
  -d '{"arm_right_shoulder_lift": 1920, "arm_right_elbow_flex": 1472}'
```

## Lock / unlock arms

Locked = torque enabled (arm holds position, sliders control it). Unlocked = torque disabled (arm goes limp, sliders show feedback only).

```bash
# Lock right arm
curl -s -X POST http://fppe:8091/lock_arm \
  -H 'Content-Type: application/json' \
  -d '{"side": "right", "lock": true}'

# Unlock left arm
curl -s -X POST http://fppe:8091/lock_arm \
  -H 'Content-Type: application/json' \
  -d '{"side": "left", "lock": false}'
```

## Preset positions

```bash
# Home position (arms folded down)
curl -s -X POST http://fppe:8091/go_home

# Arms up position
curl -s -X POST http://fppe:8091/go_arms_up

# Middle position (all joints to 2048)
curl -s -X POST http://fppe:8091/go_middle
```

These presets automatically lock both arms before moving.

## Elevator control

```bash
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "up"}'
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "down"}'
curl -s -X POST http://fppe:8091/lift -H 'Content-Type: application/json' -d '{"action": "stop"}'
```

The lift runs at a fixed velocity until stopped.

## Emergency stop

```bash
curl -s -X POST http://fppe:8091/estop
```

Disables torque on all motors immediately. Both arms go limp, lift stops.
