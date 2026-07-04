# vision — scene description for the fppe robot

A skynet-side HTTP service that gives Purple a **sense of sight**: on request it
grabs a frame from the robot's camera and runs it through
[Moondream2](https://huggingface.co/vikhyatk/moondream2) (a small vision-language
model), returning either a caption or a free-text **visual-question answer**. It
backs the voice `look` tool and is the perception prerequisite for navigation.

Like `conductor/`, this is purely additive — it's just another client of the Pi's
`viser_control` HTTP API (`fppe:8091`), reading its MJPEG camera streams.

## Run

```bash
just vision serve --preload          # HTTP service on :8102, model warm at startup
just vision look "what do you see?"   # one-shot: grab a frame + describe it (test path)
just vision look                      # one-shot caption (no question)
```

`serve` expects `viser_control.py` up on the Pi (`just viser`) for the camera feed.

## HTTP API

CORS-open, stdlib `http.server`. Default port `8102`.

| Method & path                 | Returns                                                   |
|-------------------------------|-----------------------------------------------------------|
| `GET /health`                 | `{ok, model, revision, device, loaded}`                   |
| `GET /look?q=<question>&cam=` | `{ok, text, cam, question}` — VQA if `q` set, else caption |

`cam` defaults to `cam0` (the other eye is `cam1`). Example:

```bash
curl "http://skynet:8102/look?q=how+many+people+do+you+see"
```

## Device: `auto` (GPU when it's free, else CPU)

skynet's 16 GiB GPU is shared with the speech pipeline (~6 GiB) and games, so the
default `--device auto` claims CUDA only when it has ~4 GiB of headroom and
otherwise falls back to CPU:

| Device            | one-time load | per `look`   |
|-------------------|---------------|--------------|
| CUDA (≥4 GiB free)| ~2.9 s        | **~1.0 s**   |
| CPU (GPU full)    | ~2.9 s        | ~7–18 s      |

Override with `--device cuda` (honored even under pressure — may OOM) or
`--device cpu`. **Note:** the device is chosen once at load, so if the service
starts while the GPU is full it stays on CPU until `systemctl --user restart
vision`.

## Layout

- `src/vision/server.py` — the HTTP service (`/health`, `/look`).
- `src/vision/model.py` — `Moondream`: lazy-loaded singleton, GPU/CPU device pick,
  `describe(image, question)`. Access is lock-serialized (the server is threaded).
- `src/vision/frame.py` — `grab_frame(url)`: reads the first complete JPEG out of an
  MJPEG multipart stream and decodes it with Pillow.
- `src/vision/cli.py` — `vision serve` / `vision look`.
- `deploy/` — the `vision.service` systemd unit + `vision-server.sh` wrapper.

## Run as a service

Installed as a **user systemd unit** (mirrors `speech-to-speech.service`):

```bash
install -m 755 deploy/vision-server.sh ~/.local/bin/vision-server.sh
install -m 644 deploy/vision.service   ~/.config/systemd/user/vision.service
systemctl --user daemon-reload
systemctl --user enable --now vision.service
loginctl enable-linger "$USER"        # so it autostarts on boot without a login
```

## Notes / gotchas

- **transformers is pinned `<5`.** Moondream's `trust_remote_code` modeling predates
  transformers v5's model-loading internals (v5 expects `all_tied_weights_keys`), so
  `pyproject.toml` pins `transformers>=4.44,<5`.
- **torch is cu128** (Blackwell / RTX 5080 sm_120), same as `nav/`.
- `uv.lock` is gitignored repo-wide, matching the other sub-projects.
