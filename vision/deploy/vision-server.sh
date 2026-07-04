#!/usr/bin/env bash
# Launch wrapper for the fppe scene-description (Moondream2) service under systemd.
# Mirrors ~/.local/bin/s2s-server.sh. Install to ~/.local/bin/vision-server.sh.
set -euo pipefail

export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"

cd /home/egradman/dev/fppe/vision

# --device auto: use CUDA when it has headroom (~4GB free), else CPU. Device is
# chosen once at load, so if skynet booted with the GPU full (a game running),
# it lands on CPU until the next `systemctl --user restart vision`.
exec uv run vision serve \
    --host 0.0.0.0 --port 8102 \
    --pi-base http://fppe:8091 \
    --cam cam0 \
    --device auto \
    --preload
