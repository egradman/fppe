#### Web control panel (Viser + cameras)

Build the React UI, sync to the robot, and start the server:

```
just viser
```

This runs three steps: `build-ui` (compiles `web-ui/` with Vite), `sync` (rsyncs to fppe), then starts `viser_control.py` on the robot.

Once running:
- **http://fppe:8091/** — React app with tabs for 3D controls and full-screen camera feeds
- **http://fppe:8090/** — Viser 3D UI (also embedded in the Controls tab)
- **http://fppe:8091/mjpeg/cam0**, **/mjpeg/cam1** — raw MJPEG streams

To iterate on the React app locally: `cd examples/debug/web-ui && npm run dev`. Then rebuild and redeploy with `just viser`.

#### View all motor states
```
python examples/debug/motors.py get_motors_states \
  --port /dev/ttyACM0
```
#### Control the mobile base only
```
python examples/debug/wheels.py \
   --port /dev/ttyACM0
```

#### Control the lift axis only
```
python examples/debug/axis.py \
   --port /dev/ttyACM0
```

#### Disable torque for all arm motors
```
python examples/debug/motors.py reset_motors_torque  \
  --port /dev/ttyACM0
```

#### Rotate a specific motor by ID
```
python examples/debug/motors.py move_motor_to_position \
  --id 1 \
  --position 2 \
  --port /dev/ttyACM1
```


#### Set a new motor ID
```
python examples/debug/motors.py configure_motor_id \
  --id 10 \
  --set_id 8 \
  --port /dev/ttyACM0
```


#### Reset current position as the motor midpoint
```
python examples/debug/motors.py reset_motors_to_midpoint \
  --port /dev/ttyACM1
```


#### Execute an action script on the robot arm
```
python examples/debug/motors.py move_motors_by_script \
   --script_path action_scripts/test_dance.txt  \
   --port /dev/ttyACM0
```

