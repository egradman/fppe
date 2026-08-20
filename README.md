# Purple (fppe)

A voice-controlled mobile household robot that lives in a real house. Purple is an
omniwheel base with two arms, a face, and a camera, driven by a behavior tree that
you can talk to: *"Hey Purple — what do you see? Raise your arms. Go to the
kitchen."* The interesting part isn't the hardware (cheap hobby servo arms, a
Raspberry Pi, an off-the-shelf LeKiwi base) — it's the **brain layer** stacked on
top of it: py_trees behavior-tree modes as the single owner of actuation, a VLM
that gives it an actual sense of sight, and a realtime speech-to-speech loop whose
LLM can call tools that *move the robot*, not just describe moving the robot.

**Demo video: [I Built a Voice-Controlled Mini-Person That Chases My Dogs](https://youtu.be/569O2DqMGXY)**

This repo is a fork of [`liyiteng/lerobot_alohamini`](https://github.com/liyiteng/lerobot_alohamini)
(itself a LeRobot fork for the AlohaMini dual-arm wheeled platform). The low-level
arm/base control and the LeRobot dataset/training tooling come from upstream; the
`conductor/`, `vision/`, `nav/`, `voice/`, `leader_teleop/`, `pedal_teleop/` and web-UI
layers are this project.

> Naming note: Purple is an **it**, not gendered.

---

## The robot

**Body** — a LeKiwi omniwheel base, two SO-101 arms on a motorized lift, cameras
(`cam0` narrow mono, `cam2` fisheye nav cam), a rendered face on a kiosk display, a
USB mic/speaker, a laser pointer, and a GPIO e-stop + illuminated button HMI. A
Raspberry Pi 5 on board runs everything that touches a motor.

**Brain** — a separate machine in the garage (`skynet`, RTX 5080) runs the models
and the orchestration. The two talk over Tailscale.

```
  ┌─────────────────────────── BODY: fppe (Raspberry Pi 5) ─────────────────────────┐
  │  examples/debug/viser_control.py — the single low-level control hub              │
  │    HTTP :8091   /state · /goal_position · /go_home|/go_arms_up|/go_middle        │
  │                 /lock_arm · /lift · /estop · /teleop_mode · /base_input_source   │
  │                 /gamepad · /mjpeg/cam0 · /mjpeg/cam2 · the web UI + face kiosk   │
  │    UDP  :9999 leader arms   :9998 pedals   :9997 autonomous base ("auto")        │
  │    Two enums gate actuation: teleop_mode {local,remote},                         │
  │    base_input_source {off,gamepad,pedals,auto}.  GPIO e-stop cuts motor power.   │
  └───────────────▲───────────────────────▲───────────────────────▲─────────────────┘
                  │ HTTP state + control  │ MJPEG frames          │ UDP command streams
  ┌───────────────┼───────────────────────┼───────────────────────┼─────────────────┐
  │  BRAIN: skynet (RTX 5080)             │                       │                 │
  │                                       │                       │                 │
  │   conductor :8100  ◀── owns modes ────┘   vision :8102  ◀──────┘   senders (UDP) │
  │   py_trees behavior tree                  Moondream2 VLM          leader / pedal │
  │   (the single owner of actuation)         "what do you see?"      (spawned by    │
  │        ▲                                        ▲                  the conductor)│
  │        │ POST /mission,/preset,/interrupt        │ GET /look                     │
  │        │                                        │                                │
  │   speech-to-speech :8765 ── realtime voice ── listen_and_play_realtime.py         │
  │   (STT ▸ LLM ▸ TTS)          + LLM tool calls   (the voice client, runs on the Pi)│
  │   nav-serve :8107 (ViNT visual nav)                                              │
  └──────────────────────────────────────────────────────────────────────────────────┘

     Human interfaces:  web UI + face kiosk (from the Pi) · voice ("Hey Purple…") · phone
```

The design rule the whole system follows: **anything that changes what the robot is
*doing* goes through the conductor; anything that only *observes* bypasses it.** Nothing
in `viser_control.py` had to change to add the brain — conductor, vision and voice are
all just clients of its small HTTP/UDP surface.

Full write-up: **[ARCHITECTURE.md](ARCHITECTURE.md)**.

---

## What it does

**Behavior-tree modes** (`conductor/`, [README](conductor/README.md)) — modes are
py_trees subtrees; switching a mode swaps the subtree, and the tree owns the
lifecycle of the skynet-side UDP sender processes so a mode is self-contained.
Teardown keys on py_trees' `INVALID` status, which means mode switches and the e-stop
tear resources down for free. The root is `Selector[safety, mission]`, so tripping the
GPIO e-stop structurally preempts whatever was running. `conductor serve` exposes it
on `:8100` (`/status`, `/modes`, `/catalog`, `/mission`, `/mission/dsl`, `/interrupt`,
`/preset`, `/abort`) as the one place mode changes happen — web UI, voice LLM and
`curl` all go through it.

**Vision** (`vision/`, [README](vision/README.md)) — [Moondream2](https://huggingface.co/vikhyatk/moondream2)
as a persistent service on `:8102`. `GET /look?q=…` grabs one frame off the Pi's MJPEG
stream and answers a free-text visual question (VQA, not just captioning). Picks
CUDA when the GPU has headroom (~1 s/look) and falls back to CPU (~7–18 s) when the
speech pipeline has it — device is chosen once at load, so restart the service to
re-pick.

**Voice** — a realtime speech-to-speech loop (`voice/scripts/listen_and_play_realtime.py`
on the Pi, talking to the `speech-to-speech` server on `:8765`) where the LLM has
function-calling tools: `set_mode` and `move_arms` route to the conductor, `look`
routes straight to vision, plus `remember` (a small bounded long-term memory file) and
`dance`. Purple's persona lives in [`voice/prompt.md`](voice/prompt.md) and is told to
never make up what's in the room — always `look` first. The client distinguishes
*write* tools (the model's inline acknowledgement is enough) from *read* tools (whose
result is the payload the model must speak) to avoid colliding with the server's
already-open response; see ARCHITECTURE.md for that story.

**Voice plan-building** (`conductor/planner.py`, `plan.py`, design in
[`conductor/docs/voice-planner.md`](conductor/docs/voice-planner.md)) — v1 of the
two-brain split: the speech model is the mouth, a Claude-backed planner in the
conductor is the brain that turns an utterance into validated behavior-tree DSL.
Gated by an explicit plan mode (`EMPTY → DRAFT → RUNNING`), so ordinary conversation
can't rewrite the robot's plan by accident.

**Teleop** — leader-arm teleop (`leader_teleop/`, two SO-101 leader arms streaming
joint angles over UDP `:9999`), foot-pedal base teleop (`pedal_teleop/`, IMU body
commands on `:9998`), browser-gamepad teleop through the web UI, and a phone page
(`examples/debug/teleop.html`) that aims the right arm and laser using the phone's
motion sensors.

**Web UI + face** (`examples/debug/web-ui/`, [debug README](examples/debug/README.md)) —
a React app served from the Pi at `:8091` with tabs for 3D controls, camera feeds,
recording, and Purple's animated face. The face pane's status card is rendered by the
conductor (`GET /status/markdown`). `conductor viz` (`:8110`) is a React Flow tree
viewer paired with the same face component, so you can watch the compiled tree and the
face react together.

**AprilTag docking** (`conductor/behaviors/tags.py`, `tags.py`) — a 4-DoF visual servo
(`search_for_tag` → `servo_to_tag` → `position_at_tag`) that drives the holonomic base
onto a tag36h11 marker for the last meter. Built and validated on hardware.

**Navigation** (`nav/`, [README](nav/README.md)) — ViNT visual navigation, vendored from
`robodhruv/visualnav-transformer`, served on `:8107` and driven by the conductor through
the shared `auto` base source. Phase 2 (drive to a single goal photo) works on hardware.
Room-to-room navigation over a labeled map is still being designed — see
[`docs/mapping/map.md`](docs/mapping/map.md).

### Honest status

| Thing | State |
|---|---|
| `idle`, `teleop`, `local_teleop` modes | working |
| e-stop preemption + sender teardown | working |
| vision `/look` (VQA) | working |
| voice tools (`set_mode`, `move_arms`, `look`, `remember`, `dance`) | working |
| voice plan-builder (Claude → DSL) | v1, plan-mode gated |
| AprilTag dock (`search_and_dock`, `dock_test`) | working on hardware |
| `nav_to_goal` (ViNT, single goal photo) | works on hardware |
| `chase_dogs` | reactive tree is real and offline-tested; the open-vocab detector behind `object_visible` is a pluggable seam — with nothing wired it reports "not seen" and the mode degrades to an endless search |
| `nav` (named destinations), `clean_room` | **stubs** — wired into the DSL, return an honest `FAILURE` |
| `run_act` (imitation-learned arm policies) | **simulated** — a known action returns immediate `SUCCESS`; there's no ACT inference runner yet. The upstream LeRobot record/train/eval pipeline for producing those policies does work (see below) |
| metric-nav house map (LingBot) | design docs only, nothing built |

---

## Getting started / developing

This is not a general-purpose kit — the hosts (`fppe`, `skynet`), ports and device
paths are hard-coded for one robot. But the workflow is all in the [`justfile`](justfile):

```bash
just sync              # rsync the repo to the Pi (fppe:lerobot_alohamini/)
just viser             # build the web UI, sync, and run viser_control.py in the foreground
just bounce            # build + sync + restart the *autostarted* viser_control (routine deploys)
just kiosk             # relaunch the Pi's kiosk browser on the face tab
just web-dev           # local Vite HMR for the web UI; http://localhost:3000/?dev drives the face with no backend
just remote <cmd>      # run a command in the conda env on the Pi

just conductor list|show <mode>|run <mode>|serve|viz    # the behavior tree
just vision serve --preload | just vision look "…"      # the VLM service
just leader-teleop | just pedal-teleop                  # UDP senders (usually spawned by the conductor)
just record-dance <name>                                # capture an arm routine for the `dance` behavior
```

Each sub-project is a `uv` project with its own `pyproject.toml` and tests
(`cd conductor && uv run pytest` runs fully offline against fakes — no hardware).

Where to read next:

- [ARCHITECTURE.md](ARCHITECTURE.md) — how conductor, vision and voice compose
- [conductor/README.md](conductor/README.md) — modes, the DSL, the two invariants, the viz app
- [vision/README.md](vision/README.md) — the `/look` API and the GPU/CPU device story
- [nav/README.md](nav/README.md) — ViNT setup and the phase plan
- [examples/debug/README.md](examples/debug/README.md) — the web control panel and the raw motor/base/lift debug commands
- [deploy/systemd/README.md](deploy/systemd/README.md) — the brain's services
- [docs/mapping/](docs/mapping/) — the in-progress metric-nav design

### Upstream: alohaibase / lerobot_alohamini

Low-level dual-arm control, calibration, dataset recording, ACT training and
evaluation come from the upstream fork. If you have AlohaMini hardware and want the
install/calibrate/record/train walkthrough (conda env, serial-port permissions,
`lerobot_host`, `teleoperate_bi.py`, `record_bi.py`, `lerobot-train`), use upstream's
docs: **<https://github.com/liyiteng/lerobot_alohamini>**. Those commands still work in
this tree — they're just not what this repo is about, so they no longer live in this
README.

---

## Deploy

**Brain (skynet).** The four services run as user systemd units grouped under a single
`purple.target`, canonical copies in [`deploy/systemd/`](deploy/systemd):

```bash
deploy/systemd/install.sh              # symlinks units + wrappers into $HOME, idempotent
systemctl --user start purple.target   # speech-to-speech :8765, vision :8102, nav-serve :8107, conductor :8100
systemctl --user stop  purple.target
journalctl --user -u conductor -f
```

Each service is `PartOf=`/`WantedBy=` the target, `nav-serve` is ordered before
`conductor`, and none hard-depend on the Pi — they retry or report offline when `fppe`
is down. The target is deliberately **not** enabled on `default.target`: you bring
Purple's brain up on purpose, not at power-on.

**Body (fppe).** `viser_control.py --voice` is launched by the Pi's labwc/wayland
autostart (which also opens the kiosk browser) — there is no systemd unit for it. Deploy
with `just sync`, restart with `just bounce` (which kills only the Python child so the
supervisor respawns it; a bare `pkill -f viser_control.py` kills the supervisor too and
nothing comes back). Autostart's failure mode is documented in [autostart.md](autostart.md).

---

## License & acknowledgments

Apache 2.0, inherited from LeRobot — see [LICENSE](LICENSE).

- [`liyiteng/lerobot_alohamini`](https://github.com/liyiteng/lerobot_alohamini) — the fork this
  repo is built on, and the source of all low-level dual-arm control
- [LeRobot](https://github.com/huggingface/lerobot) (Hugging Face) — the robotics stack underneath that
- AlohaMini / LeKiwi — the hardware design (wheeled dual-arm platform, omniwheel base)
- [py_trees](https://py-trees.readthedocs.io) — the behavior-tree library the conductor is built on
- [Moondream2](https://huggingface.co/vikhyatk/moondream2) — the small VLM behind Purple's eyes
- [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) (GNM/ViNT/NoMaD, MIT) —
  the navigation policies vendored into `nav/`
