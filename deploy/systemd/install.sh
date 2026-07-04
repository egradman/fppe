#!/usr/bin/env bash
# Install Purple's skynet-side systemd units + launch wrappers by symlinking this
# repo's canonical copies into the user systemd + local-bin locations. Idempotent,
# so re-run it after editing anything here. Run on skynet as the egradman user.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DST="$HOME/.config/systemd/user"
BIN_DST="$HOME/.local/bin"
mkdir -p "$UNIT_DST" "$BIN_DST"

echo "Linking wrappers -> $BIN_DST"
for f in "$REPO_DIR"/bin/*.sh; do
    chmod +x "$f"
    ln -sfn "$f" "$BIN_DST/$(basename "$f")"
done

echo "Linking units -> $UNIT_DST"
for f in "$REPO_DIR"/units/*; do
    ln -sfn "$f" "$UNIT_DST/$(basename "$f")"
done

systemctl --user daemon-reload
# Wire the four services into purple.target (creates purple.target.wants/*).
systemctl --user enable conductor.service nav-serve.service \
                        vision.service speech-to-speech.service

cat <<'MSG'

Installed. Control Purple's brain as one unit:
  systemctl --user start   purple.target      # everything up
  systemctl --user stop    purple.target      # everything down
  systemctl --user restart purple.target
  systemctl --user status  purple.target

Manual only — nothing autostarts at boot. To autostart instead:
  systemctl --user enable purple.target && loginctl enable-linger "$USER"
MSG
