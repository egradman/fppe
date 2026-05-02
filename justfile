sync:
    rsync -avz --delete --exclude='__pycache__' --exclude='node_modules' . fppe:lerobot_alohamini/
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
