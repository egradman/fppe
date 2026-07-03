# Purple voice — run the speech client on your Mac, pulling code from skynet.
#
# The code lives on skynet (edited there); this Mac just rsyncs a copy and runs
# it against the speech-to-speech server, using the Mac's own default mic +
# speaker (incl. Bluetooth — macOS/CoreAudio handles that natively).
#
# Setup on the Mac (one time):
#   - install just + uv:   brew install just uv
#   - make a folder and drop this file in it as `justfile`:
#       mkdir -p ~/purple-voice && cd ~/purple-voice
#       scp skynet:/home/egradman/dev/fppe/voice/mac.justfile ./justfile
#   - just listen
#   - first run: macOS asks your terminal for Microphone permission — allow it.
#
# DON'T run `uv sync` here. There's no project to sync — `just listen` uses
# `uv run --no-project` and fetches the few client deps on the fly. (The repo's
# voice/pyproject.toml is the heavy server package and is deliberately NOT synced.)
#
# Override any var on the CLI, e.g.:  just server=100.64.130.69 listen

skynet := "skynet"                            # ssh host alias for skynet (where the code lives)
srcdir := "/home/egradman/dev/fppe/voice"     # path to the voice/ dir on skynet
server := "skynet"   # speech server host: tailnet name, or IP (LAN 192.168.68.76 / tailnet 100.64.130.69)
port   := "8765"     # speech server port

# Excludes the server pyproject.toml (don't build it) and justfiles (so --delete
# won't touch your local one). Leaves scripts/, prompt.md, status.md.
#
# Pull the latest client code from skynet into this folder
sync:
    rsync -avz --delete \
        --exclude='__pycache__' --exclude='pyproject.toml' \
        --exclude='justfile' --exclude='*.justfile' \
        {{skynet}}:{{srcdir}}/ ./

# Binds the face-animation WS on 0.0.0.0 so the Pi's kiosk Face tab can reach it.
#
# Sync, then run the voice client here (default mic in / speaker out)
listen *args: sync
    uv run --no-project \
        --with 'openai>=2.28' --with sounddevice --with numpy --with websockets \
        python scripts/listen_and_play_realtime.py \
        --host {{server}} --port {{port}} \
        --block-mic-during-playback \
        --face-host 0.0.0.0 \
        --instructions "$(cat prompt.md)" {{args}}

# Broadcast status.md to the face tab's status pane (ws :8767)
status:
    uv run --no-project --with websockets \
        python scripts/status_server.py --host 0.0.0.0 --file status.md
