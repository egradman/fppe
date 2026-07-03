import { useEffect, useRef, useState } from "react";
import "./App.css";

const HOST = window.location.hostname;
const API_PORT = 8091;
const VISER_PORT = 8090;
const API_BASE = `http://${HOST}:${API_PORT}`;

const GAMEPAD_DEADZONE = 0.1;
const applyDeadzone = (vals: number[]): number[] =>
  vals.map((v) => (Math.abs(v) < GAMEPAD_DEADZONE ? 0 : v));

type Tab = "controls" | "local-teleop" | "remote-teleop" | "cam0" | "cam1";

function App() {
  const [activeTab, setActiveTab] = useState<Tab>("controls");

  const tabs: { id: Tab; label: string }[] = [
    { id: "controls", label: "Controls" },
    { id: "local-teleop", label: "Local Teleop" },
    { id: "remote-teleop", label: "Remote Teleop" },
    { id: "cam0", label: "Camera 0" },
    { id: "cam1", label: "Camera 1" },
  ];

  return (
    <div className="app">
      <EstopBanner />
      <BaseInputSelector />
      <nav className="tab-bar">
        {tabs.map((tab) => (
          <button
            key={tab.id}
            className={`tab ${activeTab === tab.id ? "active" : ""}`}
            onClick={() => setActiveTab(tab.id)}
          >
            {tab.label}
          </button>
        ))}
      </nav>
      <div className="tab-content">
        {activeTab === "controls" && (
          <iframe
            src={`http://${HOST}:${VISER_PORT}`}
            className="full-frame"
            title="Viser Controls"
            allow="autoplay; fullscreen; webgl"
          />
        )}
        {activeTab === "local-teleop" && <LocalTeleopPanel />}
        {activeTab === "remote-teleop" && <RemoteTeleopPanel />}
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

// Explicit-enable selector for which input source owns the wheels + lift.
// Mirrors the backend's BaseInputSource state; default is "off" so nothing
// drives the base until the operator deliberately picks a source.
function BaseInputSelector() {
  const [value, setValue] = useState<string>("off");

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${API_BASE}/state`);
        const j = await r.json();
        if (!cancelled) setValue(String(j.base_input_source ?? "off"));
      } catch {
        // ignore — EstopBanner already surfaces unreachable
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  const select = (next: string) => {
    setValue(next); // optimistic — poll will reconcile
    fetch(`${API_BASE}/base_input_source`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ value: next }),
    }).catch(() => {});
  };

  const options: { id: string; label: string }[] = [
    { id: "off", label: "Off" },
    { id: "gamepad", label: "Gamepad" },
    { id: "pedals", label: "Pedals" },
  ];

  return (
    <div className="base-input-selector">
      <span className="base-input-label">Base driver:</span>
      {options.map((o) => (
        <button
          key={o.id}
          className={`base-input-btn ${value === o.id ? "active" : ""}`}
          onClick={() => select(o.id)}
        >
          {o.label}
        </button>
      ))}
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
        if (now - lastSend > 33) {
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

  return { gamepadName, axes, buttons, hz };
}

function LocalTeleopPanel() {
  const [enabled, setEnabled] = useState(false);
  const { gamepadName, axes, buttons, hz } = useGamepadPump(enabled);

  // Tell the backend the operator selected local teleop. The backend
  // gates the gamepad/UDP paths on this so only one input source has
  // authority at a time.
  useEffect(() => {
    fetch(`${API_BASE}/teleop_mode`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "local" }),
    }).catch(() => {});
  }, []);

  const lx = axes[0] ?? 0;
  const ly = axes[1] ?? 0;
  const rx = axes[2] ?? 0;
  const ry = axes[3] ?? 0;

  return (
    <div className="teleop">
      <div className={`teleop-status ${enabled ? "on" : "off"}`}>
        Teleop {enabled ? "ACTIVE" : "disabled"} — {gamepadName}
      </div>

      <label className="teleop-toggle">
        <input
          type="checkbox"
          checked={enabled}
          onChange={(e) => setEnabled(e.target.checked)}
        />
        Enable teleop (left stick = strafe/forward, right stick L/R = rotate,
        buttons 12/13 = lift)
      </label>

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
