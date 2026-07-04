#!/usr/bin/env bash
# Launch wrapper for the behavior-tree conductor under systemd.
# Mirrors ~/.local/bin/vision-server.sh / s2s-server.sh. Part of purple.target.
set -euo pipefail

export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"

cd /home/egradman/dev/fppe/conductor

# Boots in idle (base off, arms local); the HTTP API on :8100 drives mode
# switches. Talks to the Pi at fppe:8091 and reads nav commands from nav-serve
# on :8107. Both degrade gracefully (retry / report offline) if down.
exec uv run conductor serve \
    --mode idle \
    --host fppe \
    --api-port 8091 \
    --port 8100 \
    --nav-url http://localhost:8107
