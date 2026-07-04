#!/usr/bin/env bash
# Launch wrapper for the visual-nav (ViNT) service under systemd.
# Mirrors ~/.local/bin/vision-server.sh. Part of purple.target.
set -euo pipefail

export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"

cd /home/egradman/dev/fppe/nav

# Holds ViNT + the fisheye reader loaded; POST /goal, GET /command. Sends no UDP
# to the robot — the conductor's ctx.base is the single writer. First run loads
# the ViNT checkpoint + CUDA warmup, hence the generous TimeoutStartSec.
exec uv run nav-serve \
    --robot-host fppe \
    --cam-port 8091 \
    --cam cam2 \
    --serve-port 8107 \
    --device auto
