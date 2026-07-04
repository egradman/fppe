# nav — Phase 2→4 plan (language nav on the fppe base)

Forward-looking plan. Phase 1 (offline ViNT inference harness) is **done** — see
`README.md`. This picks up from a passing **fisheye camera / FOV check** and carries
through to "Purple, go to the kitchen." Approach is LM-Nav-style topological nav on
Berkeley's ViNT (see `memory/project_nav_stack.md` for why this family).

Purple is an **it**, not gendered.

## North star (the capstone demo)

> **Drive from the library to the kitchen, park at a chore station, and put an object
> in a bowl.**

One continuous behavior that exercises every layer: topological nav (library→kitchen)
→ AprilTag search-and-dock (the precise "last meter") → an ACT manipulation policy
(object → bowl). As a conductor tree it's roughly:

```
Sequence[ navigate_to("kitchen"),        # Phase 2–4: topological nav
          search_and_dock(tag="kitchen_station"),  # Phase 5: AprilTag localization
          run_skill("object_in_bowl") ]  # Phase 6: trained ACT policy
```

Everything below builds toward this, in order. Nothing past Gate 0 is tomorrow's work.

## The mental model (read this first)

The map is a **photo album, not a floorplan**. No SLAM, no coordinates, no occupancy
grid — fppe has no depth or odometry, which is *why* we chose topological nav.

- **Nodes** = camera frames captured while driving ("what the world looked like here").
- **Edges** = "you can get from frame A to frame B, ~N steps apart" — the weight is
  ViNT's temporal-distance head (the scalar the Phase 1 harness already prints).
- **Navigation** = ViNT repeatedly answers "which way moves me from what-I-see-now
  toward the goal frame"; graph search chains those hops across the album.

Consequences that shape everything below:
- The graph is **only as connected as the driving**. Junctions (hallways, doors, the
  open kitchen/dining/tv core) must be physically driven through, or routes can't branch
  and "go to X" fails even though a frame of X exists.
- It is **open-loop and image-relative** — brittle to big appearance change (lights
  on/off, furniture moved, day/night) and it does **not** avoid obstacles (a dog in the
  path). Those are limits of this family, not bugs to fix in Phase 4.

## Gate 0 — fisheye / FOV check (do this first, tomorrow)

ViNT trains on a ~170° FOV. fppe's current webcams are much narrower — the **main
quality risk**. Before building anything:

1. Mount the ordered fisheye USB cam; confirm it shows up as an MJPEG feed from
   `viser_control` (`http://fppe:8091/mjpeg/camN`).
2. Point-and-shoot sanity: capture a short forward-driving clip through the fisheye and
   run the Phase 1 harness on it (`uv run nav-harness --media <clip>.mp4`). Forward video
   should still yield net-forward waypoints, and the distance head should respond.
3. **Decision point:** if the fisheye feed produces sane, more-confident waypoints than
   the narrow cam did → gate passes, proceed to Phase 2. If not, stop and reconsider
   (crop/FOV tuning, or a different goal-image strategy) before investing in the executor.

## Phase 2 — the executor: one real hop (`base_input_source == "nav"`)

Smallest honest milestone on real hardware: **drive to a single goal photo.** No graph
yet.

- **viser_control (`examples/debug/viser_control.py`):** add `"nav"` to
  `BaseInputSource._VALUES` and a `start_nav_udp_listener` mirroring
  `start_pedal_udp_listener` (the `:9997` seam noted in ARCHITECTURE.md). It must feed
  the existing **300 ms wheel watchdog** — stop streaming → wheels stop.
- **nav executor (new, in `nav/`):** loop at a few Hz —
  1. grab the live fisheye frame + the last few as context,
  2. run ViNT `(context, goal_photo)` → waypoints (reuse `run_vint` from `harness.py`),
  3. proportional controller: waypoints → body velocity `(vx, vy, ω)`. The holonomic
     omni base can **strafe straight** at the waypoint, which keeps the controller simple.
  4. stream velocities as a `PedalPacket`-shaped UDP datagram to the new listener.
  5. stop when ViNT's distance-to-goal drops below a threshold.
- **Verify:** put Purple in a room, hand it a photo taken a few meters ahead, watch it
  drive there. **Hand on the physical e-stop for the first base-moving run.** This is the
  first time the fisheye + controller + watchdog run as a closed loop.

Deliberately **not** in Phase 2: the graph, labels, language. One hop, proven.

## Phase 3 — record the house (teleop tour + label-while-driving)

Now build the album. Reuses the teleop path already in place.

**The drive:** a single **continuous tour** — not out-and-back dead ends — that passes
through every room *and every junction between them*, closing loops where paths meet
(hallways, the kitchen/dining/tv core). You (Eric) drive; Dash labels from a phone.

**Label-while-driving UI (decided):** a **phone-friendly web page** served from the Pi's
`viser_control` (same origin as the MJPEG feeds — no CORS, works over LAN/Tailscale):
- shows the **live camera feed** (so Dash can see when Purple is "in" a room), and
- a grid of **big buttons, one per label** (the 14 below).
- A tap emits a `(timestamp, label)` event; the recorder attaches it to whatever frame
  was live at that instant. **Multiple taps per room are encouraged** — each becomes an
  extra candidate goal node, so nav can head for whichever instance is nearest.

**Recorder:** logs the frame stream + the interleaved label events. **Offline afterward**:
subsample frames into nodes (drop near-duplicates while stationary, ~one node per meter
of travel), and fill edges by running ViNT's distance function between nearby frames →
the topological graph. Labeled frames become named goal nodes.

### Rooms to capture (14 labels — from `memory/project_nav_rooms.md`)

laundry room · master bedroom · master bathroom · master hallway · library ·
dining room · kitchen · front door · back door · tv area · dash's hallway ·
mom's office · dash's bedroom · dash's bathroom

Layout notes for planning the tour:
- **Master suite:** master bedroom → master bathroom (bathroom likely *through* the
  bedroom) → master hallway as the connector.
- **Dash's wing:** dash's hallway → dash's bedroom → dash's bathroom (hallway is the spine).
- **Common core:** kitchen ↔ dining room ↔ tv area ↔ library ↔ front door (open, mutually
  adjacent) with back door hanging off it.
- **Outliers:** laundry room, mom's office — note where they branch off the main route.
- The 3 bathrooms + 2 bedrooms **look alike** → explicit labels (not CLIP) are what
  disambiguate them.

## Phase 4 — language layer ("go to the kitchen")

- **Named lookup (primary):** `navigate_to("kitchen")` → the labeled goal node(s) →
  graph search (pick nearest instance to current position-in-graph) → hand the goal
  frame to the Phase 2 executor. Reliable, because the labels are explicit.
- **CLIP (fallback):** for places not explicitly named, CLIP scores node images against
  the phrase and proposes candidates. Fallback only — CLIP can't tell the bathrooms apart.
- **LLM parsing:** the existing voice LLM (already has `set_mode`, `move_arms`, `look`)
  parses instructions into a sequence of landmarks ("kitchen then the front door" → two
  goal nodes → concatenated route).
- **Conductor integration:** `navigate_to` becomes a real conductor **mode**, filling the
  `nav` stub the conductor already holds open (`conductor/`). It slots into the same
  single-owner-of-actuation model as every other mode — the voice LLM asks the conductor,
  the conductor drives the executor.

## Phase 5 — AprilTag search-and-dock (the "last meter")

Topological nav lands the base **approximately** in a room (~half a meter, arbitrary
heading). A manipulation policy needs the base placed to **centimeters** from a
consistent approach pose. AprilTags close that gap deterministically — classical CV, no
training, full 6-DOF pose (range + bearing + orientation) to the tag.

- **Infrastructure:** stick an AprilTag at each chore station (kitchen counter, etc.).
  Tags double as semantic anchors (tag id → station) and can verify "am I actually here"
  after nav.
- **Behavior (a small conductor subtree):**
  ```
  Sequence[ SearchForTag(id, timeout),   # rotate/scan; RUNNING until detected; FAILURE on timeout (bounded)
            ServoToTag(id, offset) ]      # drive to a target standoff; RUNNING until within tolerance
  ```
  Both obey the non-blocking tick contract (side effects in `initialise`, `RUNNING` until
  the polled state confirms).
- **Why the omni base makes this easy:** the holonomic LeKiwi base decouples the three
  errors — **strafe** kills lateral offset, **translate** kills range, **rotate** kills
  heading — independently. The controller is three proportional terms nulling three
  numbers the detector already returns. (A diff-drive base would make this painful.)
- **Depends on** the same fisheye **camera calibration** nav needs (pose accuracy ∝ good
  intrinsics; fisheye distorts most at the edges, so keep the tag near image center for
  the final approach, or servo on a rectified crop).
- **Tooling:** `pupil_apriltags` / the AprilTag lib; known tag size + intrinsics → pose.

The payoff: every skill now starts from the **same** tag-relative pose, so the Phase 6
policy never has to tolerate base-position variation — train from one viewpoint, run
reliably.

## Phase 6 — ACT manipulation skill ("put the object in the bowl")

The chore itself. `lerobot_alohamini` is already a manipulation-imitation stack — leader
arms, `record_bi.py`, ACT training — so this reuses the pipeline that ships in this repo.

- **Record:** teleop the task ~dozens of times **from the docked pose** (Phase 5 gives a
  repeatable start), logging to a LeRobot dataset (`record_bi.py`).
- **Train:** ACT policy (`lerobot-train --policy.type=act`), optionally augmented with
  Isaac Sim demos (skynet IL setup — see `memory/project_isaac_setup.md`).
- **Wrap:** the trained checkpoint becomes a conductor `run_skill("object_in_bowl")` leaf.
  The behavior-tree node is model-agnostic — swap ACT → a VLA later without touching the
  tree.
- **Why ACT before a VLA:** ACT is reliable, narrow, and trainable per task *today*; a
  language-conditioned VLA (π0 / GR00T / RDT) is the "one model for many chores" upgrade
  once maintaining N per-task policies gets annoying. Start narrow-and-working.

## Tomorrow's concrete order

1. **Gate 0** — mount fisheye, confirm MJPEG feed, run harness on a fisheye clip, decide.
2. If gate passes → **Phase 2**: `base_input_source == "nav"` + nav UDP listener + the
   executor loop; verify single-goal-photo drive with a hand on the e-stop.
3. Only then → **Phase 3** recording UI + tour (needs a working `nav` source to exist).

Everything past Gate 0 is plan, not code. Stop at each gate and eyeball the result before
investing in the next stage.
