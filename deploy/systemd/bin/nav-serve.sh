#!/usr/bin/env bash
# Launch wrapper for the visual-nav (ViNT) service under systemd.
# Mirrors ~/.local/bin/vision-server.sh. Part of purple.target.
set -euo pipefail

export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"

cd /home/egradman/dev/fppe/nav

# Holds ViNT + the fisheye reader loaded; POST /goal|/route, GET /command. Sends
# no UDP to the robot — the conductor's ctx.base is the single writer. First run
# loads the ViNT checkpoint + CUDA warmup, hence the generous TimeoutStartSec.
# --route preloads the garage topological map for navigate_to(label) breadcrumb
# following (Phase 3); node images live in the route dir, so it's self-contained.
exec uv run nav-serve \
    --robot-host fppe \
    --cam-port 8091 \
    --cam cam2 \
    --serve-port 8107 \
    --device auto \
    --route routes/garage/route.json \
    --heading-gain 0.6 \
    --max-omega 40
# Tuned on the garage run (2026-07-04): default yaw (0.3/20) tracked the curve but
# cut wide; 0.6/40 hugs it. Adjust live without a reload via POST :8107/gains.
