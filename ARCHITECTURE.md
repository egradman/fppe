# fppe — Brain-Side Architecture (Conductor · Vision · Voice)

This document describes the **orchestration, perception, and conversational-control
layer** built on top of the `lerobot_alohamini` fork for the **fppe** robot
("Purple"). It is purely additive: the robot's low-level control hub
(`examples/debug/viser_control.py`) is unchanged, and everything here layers on
its HTTP/UDP surface.

Purple is a mobile household robot — an omniwheel (LeKiwi) base, two SO-101 arms
on a lift, cameras, and a voice. This layer is what turns that hardware from
"a human flips enums and launches senders by hand" into **named modes you can
switch, a real sense of sight, and a voice assistant that can actually act.**

> Naming note: Purple is an **it**, not gendered.

---

## The brain / body split

Everything runs across two machines connected over Tailscale:

```
  ┌─────────────────────────── BODY: fppe (Raspberry Pi) ───────────────────────────┐
  │  examples/debug/viser_control.py — the single low-level control hub               │
  │    HTTP :8091   /state · /goal_position · /go_home|/go_arms_up|/go_middle          │
  │                 /lock_arm · /lift · /estop · /teleop_mode · /base_input_source     │
  │                 /gamepad · /mjpeg/cam0 · /mjpeg/cam1                               │
  │    UDP  :9999 leader arms   :9998 pedal base   (:9997 nav, future)                 │
  │    Two enums gate actuation: teleop_mode {local,remote}, base_input_source         │
  │    {off,gamepad,pedals,(nav)}.  GPIO e-stop cuts motor power.                      │
  └───────────────▲───────────────────────▲───────────────────────▲──────────────────┘
                  │ HTTP/state+control     │ MJPEG frames          │ UDP command streams
  ┌───────────────┼───────────────────────┼───────────────────────┼──────────────────┐
  │  BRAIN: skynet (RTX 5080)             │                       │                    │
  │                                        │                       │                    │
  │   conductor  :8100  ◀── owns modes ───┘   vision  :8102  ◀─────┘   senders (UDP)    │
  │   behavior tree                            Moondream2 VLM          leader / pedal    │
  │   (the single owner                        "what do you see?"      (spawned by the   │
  │    of actuation)                                                    conductor)       │
  │        ▲                                        ▲                                    │
  │        │  POST /mission,/preset                 │  GET /look                         │
  │        │                                        │                                    │
  │   speech-to-speech :8765 ── realtime voice ── listen_and_play_realtime.py            │
  │   (STT ▸ LLM ▸ TTS)          + LLM tool calls   (the voice client, runs on the Pi)   │
  └──────────────────────────────────────────────────────────────────────────────────┘

           Human interfaces:  Web UI (served from the Pi)  ·  Voice ("Hey Purple…")
```

The three things we built — **conductor**, **vision**, **voice tooling** — are the
brain-side boxes. The rest of this doc takes them one at a time, then shows how
they compose.

---

## 1. Conductor — behavior-tree modes

**`conductor/`** ([full README](conductor/README.md)) is a
[py_trees](https://py-trees.readthedocs.io) behavior tree that runs on skynet and
acts as a client of the Pi's `:8091`. It answers one question cleanly: *what is the
robot doing right now, and how do we switch that safely?*

- **Modes are subtrees.** `idle`, `teleop`, `local_teleop` are real; `nav` and
  `clean_room` are honest stubs. Switching a mode swaps the subtree.
- **The tree owns the sender processes.** Entering `teleop` starts the skynet-side
  `leader-teleop` / `pedal-teleop` UDP senders; leaving it kills them. A mode is
  therefore *self-contained* — nothing leaks across a switch.
- **Two invariants make that work:**
  1. **Non-blocking ticks** — side effects fire in `initialise()`, and `update()`
     returns `RUNNING` until the polled `/state` snapshot confirms the effect.
  2. **Teardown keys on `INVALID`** — a preempted subtree is stopped with
     `INVALID`, which is when `StartSender` kills its process. Mode switches and the
     e-stop tear everything down for free.
- **Safety is structural.** The root is `Selector[safety, mission]`; when the GPIO
  e-stop trips, the safety branch wins and preempts the mission (senders die, base
  source → off). On release the mission re-initialises.

### The HTTP service — one owner of mode switching

`conductor serve` runs the engine as a persistent service (stdlib http.server,
CORS-open) so that **the behavior tree is the single owner of what the robot is
doing.** The web UI, the voice LLM, and `curl` all ask the conductor for a change
instead of poking `viser_control` directly:

| Method & path        | Effect                                                        |
|----------------------|---------------------------------------------------------------|
| `GET /status`        | current mission, modes, e-stop, robot state, rendered tree    |
| `GET /modes`         | list of named modes                                           |
| `GET /catalog`       | the LLM-facing behavior catalog (for future tree composition) |
| `POST /mission`      | `{name}` — load a named mode                                  |
| `POST /mission/dsl`  | `{name?, dsl}` — load an arbitrary tree (LLM/custom)          |
| `POST /interrupt`    | `{name?, dsl}` — run a tree now, **auto-restore** prior mode  |
| `POST /preset`       | `{preset}` — move arms to a posture, then auto-restore        |
| `POST /abort`        | drop to `idle`                                                |

`/interrupt` and `/preset` ride `Engine.interrupt`: the activity runs immediately
and the previous mode is restored from a resume stack when it succeeds. `/preset`
builds a `go_preset → wait_settled` sequence (presets lock the arms themselves and
aren't gated by `teleop_mode`, so they work from any mode). The **web UI**'s mode
selector polls `/status` and POSTs `/mission`+`/abort`.

---

## 2. Vision — a real sense of sight

**`vision/`** ([README](vision/README.md)) is Purple's answer to *"what do you
see?"* — the prerequisite we built before navigation. It runs
[Moondream2](https://huggingface.co/vikhyatk/moondream2) as a persistent HTTP
service on skynet (`:8102`):

- `GET /look?q=<question>&cam=cam0` grabs **one** JPEG off the Pi's MJPEG stream
  and runs the VLM. With no `q` it returns a caption; with a `q` it answers a
  free-text **visual question** ("how many people?", "what's on the floor?"). VQA,
  not just captioning, is what makes it an actual sense rather than a party trick.
- `GET /health` reports device and load state.

**Device is `auto`.** skynet's 16 GiB GPU is shared with the speech pipeline (~6 GiB)
and Dash's games, so `auto` claims CUDA only when it has ~4 GiB of headroom
(**~1 s/look**) and otherwise falls back to CPU (**~7–18 s/look**). It ships as a
managed **user systemd service** (`vision.service`, `linger`'d so it autostarts on
boot, `--preload` so the model is warm). See `vision/deploy/`.

> Caveat: device is chosen once at load. If the service starts while the GPU is
> full, it lands on CPU and stays there until `systemctl --user restart vision`.

---

## 3. Voice tooling — an assistant that can act

**`voice/scripts/listen_and_play_realtime.py`** is the realtime voice client (it
runs on the Pi, launched by `viser_control --voice`). It speaks the OpenAI Realtime
protocol to the `speech-to-speech` server on skynet (`:8765`: Parakeet STT ▸ Gemma
LLM via HF router ▸ Qwen3 TTS). On top of plain conversation, it gives the LLM
**function-calling tools** so Purple can do things, not just talk about them.

Three tools, declared to the server in the connect-time `session.update`:

| Tool         | Routes to           | Kind  | What it does                                  |
|--------------|---------------------|-------|-----------------------------------------------|
| `set_mode`   | conductor `/mission`| write | switch behavior mode (idle / teleop / …)      |
| `move_arms`  | conductor `/preset` | write | move arms to a posture, then auto-restore     |
| `look`       | vision `/look`      | read  | describe the live camera scene / answer a VQA |

Two design principles are worth calling out, because they're the throughline of the
whole system:

**Actions go through the conductor; queries don't.** `set_mode` and `move_arms`
change what the robot is *doing*, so they hit the behavior tree — preserving its
single-owner-of-actuation invariant. `look` is a read-only query with no side
effect, so it goes **directly** to the vision service. Perception doesn't belong on
the actuation path.

**Write tools vs. read tools (the "collision" story).** The speech server emits a
tool call *inside an open response* — the model speaks its acknowledgement in the
same turn as the call. A naive client that fires `response.create` right after the
tool returns collides (`conversation_already_has_active_response`) and poisons the
session so later turns stop calling tools. The fix:

- **Write tools** (`set_mode`, `move_arms`): the inline ack is enough, so we only
  follow up to report a *failure*, and even then we defer it until the active
  response closes (`response_active` / `pending_followup` flags).
- **Read tools** (`look`): the tool *result is the payload the model must speak*, so
  `READ_TOOLS` always schedule the deferred follow-up — the model reads the scene
  description back in its own voice.

Purple's persona (`voice/prompt.md`) tells it it has these controls and eyes, to
call the tool for real actions ("don't just say you'll do it"), and to **never make
up what's in the room — always `look` first.**

---

## How they compose

The architecture is one idea applied consistently:

> **The behavior tree is the single owner of actuation. Everything that *changes
> what the robot does* — the web UI, the voice LLM, a curl — goes through the
> conductor. Everything that only *observes* (vision) bypasses it.**

A worked example — *"Purple, raise your arms and tell me what you see":*

1. The LLM calls `move_arms(arms_up)` → `POST /preset arms_up`. The conductor
   **interrupts** the current mode, runs `go_preset → wait_settled`, and the arms
   move; when the motors settle it **auto-restores** whatever mode was running.
2. The LLM calls `look()` → `GET /look`. The vision service grabs a camera frame,
   Moondream describes it, and the text comes back.
3. Because `look` is a read tool, the client schedules a follow-up response and the
   model **speaks the actual scene** back — no hallucination, no collision.

Nothing in `viser_control.py` had to change for any of this. The conductor, vision,
and voice layers are all just clients of the same small HTTP/UDP surface.

---

## Running it all

Ports, at a glance:

| Host   | Service                 | Port(s)              | Start                                   |
|--------|-------------------------|----------------------|-----------------------------------------|
| fppe   | `viser_control` (+voice)| 8091, 9999/9998 UDP  | `just viser` / labwc autostart          |
| skynet | conductor               | 8100                 | `just conductor serve` (systemd-able)   |
| skynet | vision                  | 8102                 | `just vision serve --preload` (systemd) |
| skynet | speech-to-speech        | 8765                 | `speech-to-speech.service` (systemd)    |

The skynet services (`speech-to-speech.service`, `vision.service`) run as
**user systemd units** with lingering enabled, so they come up on boot. Deploy code
to the Pi with `just sync` (rsync of the repo to `fppe:lerobot_alohamini/`); a Pi
reboot relaunches `viser_control --voice`, which reconnects the voice client so it
re-declares its tools.

## Roadmap

- **Navigation** (`nav/`) — the reason vision came first. LM-Nav/ViNT topological
  nav on the LeKiwi base; wires in as `base_input_source == "nav"` + a nav UDP
  sender, and becomes a real conductor mode. See `nav/README.md`.
- **LLM tree composition** — the conductor's `/catalog` + DSL round-trip
  (`build_tree` ⇄ `tree_to_dsl`) are the substrate for an LLM that *assembles and
  edits behavior trees at runtime*, not just picks from named modes.
