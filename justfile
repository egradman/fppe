sync:
    rsync -avz --delete --exclude='__pycache__' --exclude='node_modules' --exclude='.venv' . fppe:lerobot_alohamini/
    ssh fppe 'ln -sfn ~/lerobot_alohamini ~/fppe'

remote *args: sync
    ssh -t fppe 'source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot_alohamini && cd lerobot_alohamini && {{args}}'

wheels: (remote "python examples/debug/wheels.py --port /dev/ttyACM0")

axis: (remote "python examples/debug/axis.py --port /dev/ttyACM0")

gamepad: (remote "python examples/debug/gamepad_teleop.py --port /dev/ttyACM0")

motors: (remote "python examples/debug/motors.py get_motors_states --port /dev/ttyACM0")

test-connect: (remote "python examples/debug/test_connect.py")

calibrate: (remote "python examples/debug/calibrate.py")

go-middle: (remote "python examples/debug/go_middle.py")

move-to *args: (remote "python examples/debug/move_to.py " + args)

build-ui:
    cd examples/debug/web-ui && npm run build

viser *args: build-ui sync
    ssh fppe 'pkill -f "[v]iser_control.py"; sleep 2'
    ssh fppe 'source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot_alohamini && cd lerobot_alohamini && exec python -u examples/debug/viser_control.py {{args}}'

# Host running `just listen` (voice/mac.justfile) that serves the face-animation
# WebSockets (mouth/state/transcript on :8766, status on :8767).
face_host := "nimbus"

# Relaunch the fppe kiosk browser on the Face tab, pointing the face + status
# WebSockets at {{face_host}}. Launches into the Pi's Wayland session, detached.
# Override the host with `just face_host=air kiosk`.
# Two ssh calls (like `viser`): the bracketed [c]hromium pattern stops pkill from
# matching its own command line, and keeps the kill separate from the relaunch.
kiosk:
    ssh fppe 'pkill -f "[c]hromium.*--kiosk"; sleep 2'
    ssh fppe 'export XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus; \
      setsid chromium --ozone-platform=wayland --kiosk --noerrdialogs --disable-infobars --no-first-run \
        --password-store=basic --enable-features=Vulkan --enable-webgl --ignore-gpu-blocklist \
        "http://localhost:8091/?ws=ws://{{face_host}}:8766&status=ws://{{face_host}}:8767" \
        > /tmp/chromium-kiosk.log 2>&1 < /dev/null & echo "kiosk launched -> {{face_host}}"'

# Local (skynet) — calibrate the two SO-101 leader arms once.
leader-calibrate *args:
    cd leader_teleop && uv run leader-calibrate {{args}}

# Local (skynet) — stream leader joint angles to fppe over UDP.
leader-teleop *args:
    cd leader_teleop && uv run leader-teleop {{args}}

# Local (skynet) — stream foot-pedal IMU body commands to fppe over UDP.
pedal-teleop *args:
    cd pedal_teleop && uv run pedal-teleop {{args}}

# Local (skynet) — behavior-tree conductor: modes + sequenceable activities.
# e.g. `just conductor run teleop`, `just conductor show teleop`, `just conductor catalog`
conductor *args:
    cd conductor && uv run conductor {{args}}
