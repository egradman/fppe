# deploy/systemd — Purple's brain as a systemd unit

The four skynet-side services that make Purple's brain function, grouped under a
single **user systemd target** so they start and stop as one unit.

| Service                    | Port | What it is                                        |
|----------------------------|------|---------------------------------------------------|
| `speech-to-speech.service` | 8765 | realtime voice server (STT + LLM + TTS)           |
| `vision.service`           | 8102 | Moondream scene description (the voice `look` tool)|
| `nav-serve.service`        | 8107 | ViNT visual-nav service (`nav_to_goal`)           |
| `conductor.service`        | 8100 | behavior-tree orchestrator (modes/actuation)      |

## Use

```bash
systemctl --user start   purple.target     # everything up
systemctl --user stop    purple.target     # everything down
systemctl --user restart purple.target
systemctl --user status  purple.target
journalctl --user -u conductor -f          # tail one service
systemctl --user restart nav-serve         # bounce just one
```

## Install

```bash
deploy/systemd/install.sh
```

Symlinks the canonical copies here into `~/.config/systemd/user/` (units) and
`~/.local/bin/` (wrappers), runs `daemon-reload`, and enables the four services
into `purple.target`. Idempotent — re-run after editing anything here. **This repo
is the source of truth**; the `$HOME` locations are symlinks back to it.

## Layout

- `units/*.service`, `units/purple.target` — the systemd units. Each service is
  `PartOf=purple.target` (so `stop`/`restart` of the target propagates) and
  `WantedBy=purple.target` (so the target pulls it up). `nav-serve` is ordered
  before `conductor` (whose `NavClient` reads it).
- `bin/*-server.sh` — thin launch wrappers each unit's `ExecStart=` points at:
  `set -euo pipefail`, `cd` into the project, `exec uv run …`.

## Notes

- **Manual, not boot.** `purple.target` is intentionally not enabled on
  `default.target`, so nothing autostarts at power-on — you bring Purple up
  explicitly. To autostart instead:
  `systemctl --user enable purple.target && loginctl enable-linger "$USER"`.
- **Brain, not body.** This is Purple's brain **on skynet**. Purple's body —
  `viser_control --voice` on the Pi — is autostarted by labwc *on the Pi* and is
  not part of this target (deploy to it with `just sync`). See `../../ARCHITECTURE.md`.
- **No hard dep on the Pi.** All four retry / report offline when `fppe` is down.
- Paths in the units/wrappers are absolute and skynet/user-specific (same
  convention as the pre-existing `vision`/`speech-to-speech` units).
