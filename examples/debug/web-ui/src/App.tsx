import { useEffect, useRef, useState } from "react";
import "./App.css";
import FaceTab from "./FaceTab";

const HOST = window.location.hostname;
const API_PORT = 8091;
const VISER_PORT = 8090;
const API_BASE = `http://${HOST}:${API_PORT}`;

// The behavior-tree conductor runs on skynet and owns mode switching. The UI
// (served from the Pi) requests modes from it instead of poking the Pi's
// base_input_source directly. Override with ?conductor=host:port for other hosts.
const CONDUCTOR_BASE =
  new URLSearchParams(location.search).get("conductor")
    ? `http://${new URLSearchParams(location.search).get("conductor")}`
    : "http://skynet:8100";

const GAMEPAD_DEADZONE = 0.1;
const applyDeadzone = (vals: number[]): number[] =>
  vals.map((v) => (Math.abs(v) < GAMEPAD_DEADZONE ? 0 : v));

type Tab = "controls" | "local-teleop" | "remote-teleop" | "record" | "cam0" | "cam1" | "cam2" | "face";

// Goal labels offered while recording a tour. Garage test uses two; the house
// map expands this to the 14 rooms (see nav/PLAN.md / project_nav_rooms.md).
const TOUR_LABELS = ["inward", "outward"];

// The kiosk has no keyboard, so tour sessions auto-name themselves as a random
// two-word slug (with a 🎲 reroll button) instead of making the operator type.
const NAME_ADJECTIVES = [
  "brave", "calm", "clever", "cozy", "dusty", "eager", "fuzzy", "gentle", "happy",
  "jolly", "lucky", "mellow", "nimble", "plucky", "quiet", "rapid", "shiny",
  "snug", "spry", "sunny", "swift", "tidy", "vivid", "witty", "zesty",
];
const NAME_NOUNS = [
  "otter", "falcon", "willow", "cedar", "pebble", "meadow", "comet", "harbor",
  "lantern", "maple", "nimbus", "orchard", "quartz", "rabbit", "sparrow",
  "thistle", "walrus", "badger", "cricket", "dolphin", "ferret", "gecko",
  "heron", "ibex", "jaguar",
];
const randomTourName = (): string => {
  const a = NAME_ADJECTIVES[Math.floor(Math.random() * NAME_ADJECTIVES.length)];
  const n = NAME_NOUNS[Math.floor(Math.random() * NAME_NOUNS.length)];
  return `${a}-${n}`;
};

function App() {
  const [activeTab, setActiveTab] = useState<Tab>("face");
  const status = useConductorStatus();
  // One app-level gamepad pump, enabled whenever the tree is in local_teleop —
  // regardless of which tab is showing. The mode IS the enable signal; no toggle.
  const pump = useGamepadPump(status.mission === "local_teleop");

  const tabs: { id: Tab; label: string }[] = [
    { id: "controls", label: "Controls" },
    { id: "local-teleop", label: "Local Teleop" },
    { id: "remote-teleop", label: "Remote Teleop" },
    { id: "record", label: "Record" },
    { id: "cam0", label: "Camera 0" },
    { id: "cam1", label: "Camera 1" },
    { id: "cam2", label: "Fisheye" },
    { id: "face", label: "Face" },
  ];

  // Face mode is a chromeless full-screen view: no e-stop banner, mode selector,
  // or tab bar. Tapping anywhere on the face returns to the tabbed interface.
  if (activeTab === "face") {
    return (
      <div className="app app-face" onClick={() => setActiveTab("controls")}>
        <FaceTab />
      </div>
    );
  }

  return (
    <div className="app">
      <EstopBanner />
      <div className="top-bar">
        <div className="view-selector">
          <label className="mode-label" htmlFor="view-dd">View</label>
          <select
            id="view-dd"
            className="mode-dropdown"
            value={activeTab}
            onChange={(e) => setActiveTab(e.target.value as Tab)}
          >
            {tabs.map((tab) => (
              <option key={tab.id} value={tab.id}>{tab.label}</option>
            ))}
          </select>
        </div>
        <ModeSelector status={status} />
      </div>
      <div className="tab-content">
        {activeTab === "controls" && (
          <iframe
            src={`http://${HOST}:${VISER_PORT}`}
            className="full-frame"
            title="Viser Controls"
            allow="autoplay; fullscreen; webgl"
          />
        )}
        {activeTab === "local-teleop" && (
          <LocalTeleopPanel pump={pump} active={status.mission === "local_teleop"} />
        )}
        {activeTab === "remote-teleop" && <RemoteTeleopPanel />}
        {activeTab === "record" && <RecordPanel />}
        {activeTab === "cam0" && (
          <img
            src={`${API_BASE}/mjpeg/cam0`}
            className="full-frame camera-feed"
            alt="Camera 0"
          />
        )}
        {activeTab === "cam1" && (
          <img
            src={`${API_BASE}/mjpeg/cam1`}
            className="full-frame camera-feed"
            alt="Camera 1"
          />
        )}
        {activeTab === "cam2" && (
          <img
            src={`${API_BASE}/mjpeg/cam2`}
            className="full-frame camera-feed"
            alt="Fisheye nav cam"
          />
        )}
      </div>
    </div>
  );
}

function EstopBanner() {
  const [engaged, setEngaged] = useState(false);
  const [unreachable, setUnreachable] = useState(false);

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${API_BASE}/state`);
        const j = await r.json();
        if (!cancelled) {
          setEngaged(Boolean(j.estop_engaged));
          setUnreachable(false);
        }
      } catch {
        if (!cancelled) setUnreachable(true);
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  if (unreachable) {
    return <div className="estop-banner estop-banner-warn">Robot API unreachable</div>;
  }
  if (!engaged) return null;
  return <div className="estop-banner">E-STOP ENGAGED — motors unpowered</div>;
}

// Mode selector — the single control for robot modes now that the behavior-tree
// conductor owns mode switching. Polls the conductor's /status for the current
// mission + available modes and POSTs /mission to switch. Replaces the old direct
// base_input_source selector (the tree sets base_input_source as part of a mode).
// The voice/LLM face agent will drive these same endpoints as a tool.
type ConductorStatus = {
  mission: string;
  modes: string[];
  estop: boolean;
  unreachable: boolean;
};

// Shared poll of the conductor's /status. Drives both the mode dropdown and the
// app-level gamepad pump, so the joystick follows the active mode rather than a
// separate on-screen toggle.
function useConductorStatus(): ConductorStatus {
  const [s, setS] = useState<ConductorStatus>({
    mission: "—",
    modes: [],
    estop: false,
    unreachable: false,
  });
  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${CONDUCTOR_BASE}/status`);
        const j = await r.json();
        if (!cancelled)
          setS({
            mission: String(j.mission ?? "—"),
            modes: Array.isArray(j.modes) ? j.modes : [],
            estop: Boolean(j.estop),
            unreachable: false,
          });
      } catch {
        if (!cancelled) setS((p) => ({ ...p, unreachable: true }));
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);
  return s;
}

function ModeSelector({ status }: { status: ConductorStatus }) {
  const { mission, modes, estop, unreachable } = status;

  const select = (name: string) => {
    fetch(`${CONDUCTOR_BASE}/mission`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    }).catch(() => {});
  };

  const abort = () => {
    fetch(`${CONDUCTOR_BASE}/abort`, { method: "POST" }).catch(() => {});
  };

  const known = modes.includes(mission);
  return (
    <div className="mode-selector">
      {estop && <span className="mode-estop">E-STOP</span>}
      <label className="mode-label" htmlFor="mode-dd">Mode</label>
      <select
        id="mode-dd"
        className="mode-dropdown"
        value={known ? mission : "__current__"}
        onChange={(e) => {
          const v = e.target.value;
          if (v === "__abort__") abort();
          else if (v !== "__current__") select(v);
        }}
        disabled={unreachable}
      >
        {unreachable && <option value="__current__">conductor unreachable</option>}
        {!unreachable && !known && (
          <option value="__current__" disabled>{mission}</option>
        )}
        {modes.map((m) => (
          <option key={m} value={m}>{m}</option>
        ))}
        {!unreachable && <option value="__abort__">↺ abort → idle</option>}
      </select>
    </div>
  );
}

// Polls the browser Gamepad API at frame rate, mirrors axes/buttons into
// state, and pushes them to /gamepad at ~30 Hz. The `enabled` flag is sent
// through to the backend (the watchdog coasts wheels/lift to halt when off).
// Returns the live state for any visualizers that want to render it.
function useGamepadPump(enabled: boolean) {
  const [gamepadName, setGamepadName] = useState(
    "No gamepad detected — press a button on your controller"
  );
  const [axes, setAxes] = useState<number[]>([]);
  const [buttons, setButtons] = useState<boolean[]>([]);
  const [hz, setHz] = useState("");

  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;

  useEffect(() => {
    let rafId = 0;
    let lastSend = 0;
    let sendCount = 0;
    let lastHzStamp = performance.now();

    const onConnect = (e: GamepadEvent) => setGamepadName(e.gamepad.id);
    const onDisconnect = () =>
      setGamepadName("Gamepad disconnected — reconnect controller");
    window.addEventListener("gamepadconnected", onConnect);
    window.addEventListener("gamepaddisconnected", onDisconnect);

    const tick = () => {
      const gps = navigator.getGamepads
        ? navigator.getGamepads()
        : ((navigator as unknown as { webkitGetGamepads?: () => Gamepad[] })
            .webkitGetGamepads?.() ?? []);
      let gp: Gamepad | null = null;
      for (let i = 0; i < gps.length; i++) {
        if (gps[i]) {
          gp = gps[i];
          break;
        }
      }

      if (gp) {
        setGamepadName(gp.id);
        const a = applyDeadzone(Array.from(gp.axes));
        const b = gp.buttons.map((btn) => btn.pressed);
        setAxes(a);
        setButtons(b);

        const now = performance.now();
        if (enabledRef.current && now - lastSend > 33) {
          lastSend = now;
          fetch(`${API_BASE}/gamepad`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              axes: a,
              buttons: b,
              enabled: enabledRef.current,
            }),
          }).catch(() => {});
          sendCount++;
          if (now - lastHzStamp > 1000) {
            const rate = (sendCount / ((now - lastHzStamp) / 1000)).toFixed(1);
            setHz(`Sending at ${rate} Hz`);
            sendCount = 0;
            lastHzStamp = now;
          }
        }
      } else {
        setGamepadName(
          "No gamepad detected — press a button on your controller"
        );
      }
      rafId = requestAnimationFrame(tick);
    };
    rafId = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(rafId);
      window.removeEventListener("gamepadconnected", onConnect);
      window.removeEventListener("gamepaddisconnected", onDisconnect);
      // Tell the backend the gamepad is off when this hook unmounts so the
      // wheel/lift watchdog coasts everything to a halt promptly.
      fetch(`${API_BASE}/gamepad`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ axes: [], buttons: [], enabled: false }),
      }).catch(() => {});
    };
  }, []);

  // Coast promptly when disabled (e.g. mode switched away from local_teleop):
  // one enabled:false so the Pi's watchdog stops wheels/lift without waiting.
  useEffect(() => {
    if (!enabled) {
      fetch(`${API_BASE}/gamepad`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ axes: [], buttons: [], enabled: false }),
      }).catch(() => {});
    }
  }, [enabled]);

  return { gamepadName, axes, buttons, hz };
}

function LocalTeleopPanel({
  pump,
  active,
}: {
  pump: { gamepadName: string; axes: number[]; buttons: boolean[]; hz: string };
  active: boolean;
}) {
  const { gamepadName, axes, buttons, hz } = pump;

  const lx = axes[0] ?? 0;
  const ly = axes[1] ?? 0;
  const rx = axes[2] ?? 0;
  const ry = axes[3] ?? 0;

  return (
    <div className="teleop">
      <div className={`teleop-status ${active ? "on" : "off"}`}>
        Local teleop {active ? "ACTIVE" : "inactive — pick ‘local_teleop’ mode"} —{" "}
        {gamepadName}
      </div>
      <p className="teleop-hint">
        Left stick = strafe/forward · right stick L/R = rotate · buttons 12/13 = lift.
        Driven automatically while the tree is in <code>local_teleop</code>.
      </p>

      <div className="sticks">
        <Stick label="Left Stick" x={lx} y={ly} />
        <Stick label="Right Stick" x={rx} y={ry} />
      </div>

      <div className="panel">
        <h2>Axes</h2>
        <div className="axes-list">
          {axes.map((v, i) => (
            <div key={i} className="axis-row">
              <span className="axis-label">Axis {i}:</span>
              <span className="axis-value">{v.toFixed(3)}</span>
              <span className="axis-bar-bg">
                <span
                  className="axis-bar"
                  style={{ width: `${((v + 1) / 2) * 100}%` }}
                />
              </span>
            </div>
          ))}
        </div>
      </div>

      <div className="panel">
        <h2>Buttons</h2>
        <div className="buttons-grid">
          {buttons.map((pressed, i) => (
            <div
              key={i}
              className={`btn ${pressed ? "btn-on" : "btn-off"}`}
              title={`Button ${i}`}
            >
              {i}
            </div>
          ))}
        </div>
      </div>

      <div className="hz">{hz}</div>
    </div>
  );
}

function RemoteTeleopPanel() {
  const [mode, setMode] = useState<string>("");
  const [gamepadEnabled, setGamepadEnabled] = useState(false);
  const { gamepadName } = useGamepadPump(gamepadEnabled);

  // Switch the backend to remote teleop while this tab is mounted.
  useEffect(() => {
    fetch(`${API_BASE}/teleop_mode`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "remote" }),
    }).catch(() => {});
  }, []);

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${API_BASE}/state`);
        const j = await r.json();
        if (!cancelled) setMode(String(j.teleop_mode ?? ""));
      } catch {
        // ignore
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  const active = mode === "remote";

  const lift = (action: "up" | "down" | "stop") => {
    fetch(`${API_BASE}/lift`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action }),
    }).catch(() => {});
  };

  // Press-and-hold: send "up"/"down" on press, "stop" on release. Mirrors the
  // viser scene's btn_z_up / btn_z_down behavior.
  const holdHandlers = (action: "up" | "down") => ({
    onPointerDown: (e: React.PointerEvent) => {
      e.currentTarget.setPointerCapture(e.pointerId);
      lift(action);
    },
    onPointerUp: () => lift("stop"),
    onPointerCancel: () => lift("stop"),
    onPointerLeave: (e: React.PointerEvent) => {
      // Only stop if the pointer was actually pressed when it left.
      if (e.buttons !== 0) lift("stop");
    },
  });

  return (
    <div className="remote-teleop">
      <div className={`teleop-status ${active ? "on" : "off"}`}>
        Remote teleop {active ? "ACTIVE" : "inactive"} — UDP stream from skynet
        drives the arms
      </div>
      <div className="remote-teleop-body">
        <aside className="remote-sidebar">
          <h3>Lift</h3>
          <button className="lift-btn lift-up" {...holdHandlers("up")}>▲ Up</button>
          <button className="lift-btn lift-stop" onClick={() => lift("stop")}>■ Stop</button>
          <button className="lift-btn lift-down" {...holdHandlers("down")}>▼ Down</button>

          <h3 className="sidebar-h3-spaced">Base / wheels</h3>
          <label className="gamepad-toggle">
            <input
              type="checkbox"
              checked={gamepadEnabled}
              onChange={(e) => setGamepadEnabled(e.target.checked)}
            />
            Gamepad enabled
          </label>
          <div className="gamepad-name">{gamepadName}</div>
        </aside>
        <main className="remote-main">
          <div className="remote-cams">
            <img
              src={`${API_BASE}/mjpeg/cam0`}
              className="remote-cam"
              alt="Camera 0"
            />
            <img
              src={`${API_BASE}/mjpeg/cam1`}
              className="remote-cam"
              alt="Camera 1"
            />
          </div>
          <iframe
            src={`http://${HOST}:${VISER_PORT}`}
            className="remote-urdf-frame"
            title="Viser URDF"
            allow="autoplay; fullscreen; webgl"
          />
        </main>
      </div>
    </div>
  );
}

type RecordStatus = {
  active: boolean;
  name: string | null;
  n_frames: number;
  x: number;
  y: number;
  heading_deg: number;
  labels: { t: number; label: string; frame_i: number }[];
};

// Tour recorder UI (Phase 3). Drive a continuous teleop tour; at each stop tap a
// label button to tag the current frame as a named goal node. The Pi captures
// the nav cam + dead-reckoned pose to disk; the offline builder turns it into a
// topological route later.
function RecordPanel() {
  const [rec, setRec] = useState<RecordStatus | null>(null);
  const [name, setName] = useState(randomTourName);
  const [available, setAvailable] = useState(true);

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${API_BASE}/state`);
        const j = await r.json();
        if (cancelled) return;
        if (j.record) {
          setRec(j.record as RecordStatus);
          setAvailable(true);
        } else {
          setAvailable(false);
        }
      } catch {
        // ignore
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  const post = (path: string, body?: object) =>
    fetch(`${API_BASE}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body ?? {}),
    }).catch(() => {});

  const active = rec?.active ?? false;
  const labelCounts = (rec?.labels ?? []).reduce<Record<string, number>>((acc, l) => {
    acc[l.label] = (acc[l.label] ?? 0) + 1;
    return acc;
  }, {});

  if (!available) {
    return (
      <div className="record-panel">
        <div className="teleop-status off">
          Recorder unavailable — needs the nav cam and wheels present on the Pi.
        </div>
      </div>
    );
  }

  return (
    <div className="record-panel">
      <div className={`teleop-status ${active ? "on" : "off"}`}>
        {active
          ? `Recording '${rec?.name}' — ${rec?.n_frames} frames`
          : "Not recording — start a tour, then drive and label each room at a stop"}
      </div>
      <div className="record-body">
        <aside className="record-sidebar">
          <h3>Session</h3>
          {!active ? (
            <>
              <div className="record-name-row">
                <input
                  className="record-name"
                  type="text"
                  placeholder="name"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                />
                <button
                  className="record-reroll"
                  title="New random name"
                  onClick={() => setName(randomTourName())}
                >
                  🎲
                </button>
              </div>
              <button
                className="record-btn record-start"
                onClick={() => post("/record/start", name ? { name } : {})}
              >
                ● Start
              </button>
            </>
          ) : (
            <button
              className="record-btn record-stop"
              onClick={() => post("/record/stop")}
            >
              ■ Stop
            </button>
          )}

          <h3 className="sidebar-h3-spaced">Odometry</h3>
          <div className="record-pose">
            <div>x {rec?.x?.toFixed(2) ?? "–"} m</div>
            <div>y {rec?.y?.toFixed(2) ?? "–"} m</div>
            <div>θ {rec?.heading_deg?.toFixed(0) ?? "–"}°</div>
          </div>

          <h3 className="sidebar-h3-spaced">Label (at a stop)</h3>
          <div className="record-labels">
            {TOUR_LABELS.map((lbl) => (
              <button
                key={lbl}
                className="record-label-btn"
                disabled={!active}
                onClick={() => post("/record/label", { label: lbl })}
              >
                {lbl}
                {labelCounts[lbl] ? <span className="label-count">{labelCounts[lbl]}</span> : null}
              </button>
            ))}
          </div>
        </aside>
        <main className="record-main">
          <img
            src={`${API_BASE}/mjpeg/cam2`}
            className="full-frame camera-feed"
            alt="Nav cam"
          />
        </main>
      </div>
    </div>
  );
}

function Stick({ label, x, y }: { label: string; x: number; y: number }) {
  const r = 55;
  const px = x * r;
  const py = y * r;
  return (
    <div className="stick-wrapper">
      <label>{label}</label>
      <div className="stick">
        <div
          className="stick-dot"
          style={{
            left: `calc(50% + ${px}px)`,
            top: `calc(50% + ${py}px)`,
          }}
        />
      </div>
    </div>
  );
}

export default App;
