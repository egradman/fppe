import { useEffect, useRef, useState } from "react";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import "./FaceTab.css";

// Purple's animated face. Ported from the standalone `voice/face` Vite app into
// a native tab. Subscribes to two WebSockets served on the same host the UI is
// served from: the face-state stream (mouth envelope / state / transcript) that
// listen_and_play_realtime.py serves, and the status.md stream from
// status_server.py. Both are optional — the face shows and reconnects on its own.
const HOST = window.location.hostname || "127.0.0.1";
const FACE_PORT = 8766;
const STATUS_PORT = 8767;

type FaceState = "idle" | "listening" | "thinking" | "speaking";

function useFaceStream() {
  const [state, setState] = useState<FaceState>("idle");
  const [connected, setConnected] = useState(false);
  const [userText, setUserText] = useState("");
  const [assistantText, setAssistantText] = useState("");
  const amp = useRef(0);

  useEffect(() => {
    const url =
      new URLSearchParams(location.search).get("ws") ||
      `ws://${HOST}:${FACE_PORT}`;

    let ws: WebSocket | undefined;
    let closed = false;
    let reconnectTimer: ReturnType<typeof setTimeout> | undefined;

    const connect = () => {
      ws = new WebSocket(url);
      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        if (!closed) reconnectTimer = setTimeout(connect, 1000);
      };
      ws.onerror = () => ws?.close();
      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.t === "mouth") amp.current = msg.v;
          else if (msg.t === "state") setState(msg.v as FaceState);
          else if (msg.t === "transcript") {
            if (msg.role === "user") setUserText(msg.text);
            else if (msg.role === "assistant") setAssistantText(msg.text);
          }
        } catch {
          // ignore malformed frames
        }
      };
    };
    connect();
    return () => {
      closed = true;
      clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);

  return { state, connected, amp, userText, assistantText };
}

function useStatusStream() {
  const [markdown, setMarkdown] = useState<string | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const url =
      new URLSearchParams(location.search).get("status") ||
      `ws://${HOST}:${STATUS_PORT}`;

    let ws: WebSocket | undefined;
    let closed = false;
    let reconnectTimer: ReturnType<typeof setTimeout> | undefined;

    const connect = () => {
      ws = new WebSocket(url);
      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        if (!closed) reconnectTimer = setTimeout(connect, 1000);
      };
      ws.onerror = () => ws?.close();
      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data);
          if ("markdown" in msg) setMarkdown(msg.markdown);
        } catch {
          // ignore malformed frames
        }
      };
    };
    connect();
    return () => {
      closed = true;
      clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);

  return { markdown, connected };
}

function useBlink() {
  const [blink, setBlink] = useState(0);
  useEffect(() => {
    let stop = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const schedule = () => {
      const delay = 2500 + Math.random() * 3500;
      timer = setTimeout(async () => {
        if (stop) return;
        setBlink(1);
        await new Promise((r) => setTimeout(r, 110));
        if (stop) return;
        setBlink(0);
        schedule();
      }, delay);
    };
    schedule();
    return () => {
      stop = true;
      clearTimeout(timer);
    };
  }, []);
  return blink;
}

export default function FaceTab() {
  const { state, connected, amp, userText, assistantText } = useFaceStream();
  const { markdown, connected: statusConnected } = useStatusStream();
  const blink = useBlink();

  const smoothed = useRef(0);
  const [mouth, setMouth] = useState(0);

  useEffect(() => {
    let raf = 0;
    const tick = () => {
      const target = amp.current;
      smoothed.current += (target - smoothed.current) * 0.35;
      setMouth(smoothed.current);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [amp]);

  const eyeOpen = blink
    ? 0.05
    : state === "thinking"
    ? 0.55
    : state === "listening"
    ? 1.1
    : 1.0;
  const pupilY = state === "thinking" ? -10 : 0;
  const pupilX = 0;

  const mouthOpen = 8 + mouth * 90;
  const mouthWidth = state === "speaking" ? 110 : 90;

  // Scale the face up 30% and nudge it up the page, pivoting on the face's
  // visual center (between the eyes and mouth) so it stays framed.
  const FACE_SCALE = 1.3;
  const FACE_SHIFT_Y = -40;
  const facePivotX = 400;
  const facePivotY = 340;
  const faceTransform =
    `translate(${facePivotX} ${facePivotY + FACE_SHIFT_Y}) ` +
    `scale(${FACE_SCALE}) translate(${-facePivotX} ${-facePivotY})`;

  const hasStatus = typeof markdown === "string" && markdown.trim() !== "";
  const hasTranscript = userText !== "" || assistantText !== "";

  // Fills the tab container (.tab-content is flex:1, position:relative). When a
  // status is present the face shrinks to a corner and the markdown pane fades in.
  const faceBox = hasStatus
    ? { width: "30%", height: "40%" }
    : { width: "100%", height: "100%" };

  return (
    <div className="face-root">
      <div
        className="face-box"
        style={faceBox}
      >
        <svg
          viewBox="0 0 800 600"
          preserveAspectRatio="xMidYMid meet"
          style={{ width: "100%", height: "100%", display: "block" }}
        >
          <g transform={faceTransform}>
            <Eye cx={280} cy={250} open={eyeOpen} pupilX={pupilX} pupilY={pupilY} />
            <Eye cx={520} cy={250} open={eyeOpen} pupilX={pupilX} pupilY={pupilY} />
            <Nose cx={400} cy={370} />
            <Mouth cx={400} cy={470} rx={mouthWidth} ry={mouthOpen} />
          </g>
        </svg>
      </div>

      <div
        className="status-pane"
        style={{
          bottom: hasTranscript ? "9rem" : 0,
          opacity: hasStatus ? 1 : 0,
          pointerEvents: hasStatus ? "auto" : "none",
        }}
      >
        <ReactMarkdown remarkPlugins={[remarkGfm]}>{markdown || ""}</ReactMarkdown>
      </div>

      <div
        className="transcript-pane"
        style={{
          opacity: hasTranscript ? 1 : 0,
          transform: hasTranscript ? "translateY(0)" : "translateY(100%)",
          pointerEvents: hasTranscript ? "auto" : "none",
        }}
      >
        <TranscriptLine label="You" text={userText} color="#8fd0ff" />
        <TranscriptLine label="Purple" text={assistantText} color="#b06be0" />
      </div>

      {(!connected || !statusConnected) && (
        <div className="face-dots">
          {!connected && <Dot title="face" />}
          {!statusConnected && <Dot title="status" color="#884" />}
        </div>
      )}
    </div>
  );
}

function TranscriptLine({
  label,
  text,
  color,
}: {
  label: string;
  text: string;
  color: string;
}) {
  return (
    <div className="transcript-line">
      <span className="transcript-label" style={{ color }}>
        {label}
      </span>
      <span style={{ opacity: text ? 1 : 0.25 }}>{text || "…"}</span>
    </div>
  );
}

function Dot({ color = "#800", title }: { color?: string; title: string }) {
  return <span className="face-dot" title={title} style={{ background: color }} />;
}

function Eye({
  cx,
  cy,
  open,
  pupilX,
  pupilY,
}: {
  cx: number;
  cy: number;
  open: number;
  pupilX: number;
  pupilY: number;
}) {
  const rx = 70;
  const ry = 55 * Math.max(0.02, open);
  return (
    <g>
      <ellipse cx={cx} cy={cy} rx={rx} ry={ry} fill="#f4f4f4" />
      {open > 0.15 && (
        <circle cx={cx + pupilX} cy={cy + pupilY} r={22} fill="#111" />
      )}
    </g>
  );
}

function Nose({ cx, cy }: { cx: number; cy: number }) {
  return (
    <path
      d={`M ${cx} ${cy - 20} L ${cx - 18} ${cy + 18} L ${cx + 18} ${cy + 18} Z`}
      fill="#b06be0"
      opacity={0.9}
    />
  );
}

function Mouth({
  cx,
  cy,
  rx,
  ry,
}: {
  cx: number;
  cy: number;
  rx: number;
  ry: number;
}) {
  return (
    <ellipse
      cx={cx}
      cy={cy}
      rx={rx}
      ry={ry}
      fill="#2a0033"
      stroke="#f4f4f4"
      strokeWidth={4}
    />
  );
}
