#!/usr/bin/env bash
# Launch wrapper for the speech-to-speech realtime server under systemd.
# Reads the HF token from the huggingface cache so it isn't duplicated in a unit file.
set -euo pipefail

export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"
export HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"

cd /home/egradman/dev/speech-to-speech

exec uv run speech-to-speech \
    --mode realtime --ws_host 0.0.0.0 --ws_port 8765 \
    --stt parakeet-tdt \
    --llm_backend chat-completions \
    --tts qwen3 --qwen3_tts_backend torch \
    --model_name "google/gemma-4-31B-it:cerebras" \
    --responses_api_base_url "https://router.huggingface.co/v1" \
    --responses_api_api_key "$HF_TOKEN" \
    --responses_api_reasoning_effort none \
    --responses_api_stream \
    --enable_live_transcription
