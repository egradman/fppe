import argparse
import asyncio
import base64
import json
import time
from dataclasses import dataclass, field
from queue import Empty, Queue
from threading import Event, Lock
from typing import Any, Optional

import numpy as np
from openai import AsyncOpenAI


@dataclass
class ListenAndPlayRealtimeArguments:
    host: str = field(
        default="127.0.0.1",
        metadata={"help": "Realtime server host. Default is 127.0.0.1."},
    )
    port: int = field(
        default=8765,
        metadata={"help": "Realtime server port. Default is 8765."},
    )
    model: str = field(
        default="local",
        metadata={"help": "Model name sent to the OpenAI-compatible realtime client."},
    )
    api_key: str = field(
        default="test-key",
        metadata={"help": "API key for the OpenAI SDK client. Local server ignores it."},
    )
    base_url: Optional[str] = field(
        default=None,
        metadata={"help": "Optional HTTP base URL, e.g. http://127.0.0.1:8765/v1"},
    )
    websocket_base_url: Optional[str] = field(
        default=None,
        metadata={"help": "Optional WS base URL, e.g. ws://127.0.0.1:8765/v1"},
    )
    send_rate: int = field(
        default=16000,
        metadata={"help": "Microphone sample rate in Hz. Default is 16000."},
    )
    recv_rate: int = field(
        default=16000,
        metadata={"help": "Speaker sample rate in Hz. Default is 16000."},
    )
    chunk_size: int = field(
        default=1024,
        metadata={"help": "Audio callback block size in samples. Default is 1024."},
    )
    input_device: Optional[int] = field(
        default=None,
        metadata={"help": "Optional sounddevice input device index."},
    )
    output_device: Optional[int] = field(
        default=None,
        metadata={"help": "Optional sounddevice output device index."},
    )
    instructions: Optional[str] = field(
        default=None,
        metadata={"help": "Optional session instructions to apply on connect."},
    )
    voice: Optional[str] = field(
        default=None,
        metadata={
            "help": (
                "TTS voice sent as session.audio.output.voice. "
                "Local Kokoro: e.g. bm_fable, af_heart, am_adam. "
                "OpenAI Realtime: e.g. marin, cedar, alloy."
            ),
        },
    )
    print_json: bool = field(
        default=False,
        metadata={"help": "Print raw event payloads in addition to friendly logs."},
    )
    block_mic_during_playback: bool = field(
        default=False,
        metadata={
            "help": "If set, pause microphone capture while speaker audio is playing. Disabled by default so barge-in works."
        },
    )
    face_host: str = field(
        default="127.0.0.1",
        metadata={"help": "Host to bind the face-state WebSocket server on."},
    )
    face_port: int = field(
        default=8766,
        metadata={"help": "Port for the face-state WebSocket server. Set to 0 to disable."},
    )
    face_rate: int = field(
        default=30,
        metadata={"help": "Face mouth-envelope broadcast rate in Hz."},
    )
    conductor_host: str = field(
        default="skynet",
        metadata={"help": "Host of the conductor behavior-tree HTTP API (mode/preset tools)."},
    )
    conductor_port: int = field(
        default=8100,
        metadata={"help": "Port of the conductor HTTP API. Set to 0 to disable robot tools."},
    )
    vision_host: str = field(
        default="skynet",
        metadata={"help": "Host of the vision scene-description service (the 'look' tool)."},
    )
    vision_port: int = field(
        default=8102,
        metadata={"help": "Port of the vision service. Set to 0 to disable the look tool."},
    )


# Tools the realtime model can call to drive the robot. Both go through the
# conductor behavior tree (never the Pi directly) so the tree stays the single
# owner of what the robot is doing. Modes are refreshed from GET /modes at
# connect; this is the fallback if the conductor is unreachable then.
DEFAULT_MODES = ["idle", "teleop", "local_teleop", "nav", "clean_room", "chase_dogs"]
PRESETS = ["home", "middle", "arms_up"]

# Tools whose *result* carries information the model must speak back (a read/query,
# not a fire-and-forget action). For these we always follow up with a response so
# the model verbalizes the tool output; write tools (set_mode/move_arms) only
# follow up on failure, since their success is acknowledged inline (see the
# collision note in handle_tool_call).
READ_TOOLS = {"look"}


def _conductor_base(args: "ListenAndPlayRealtimeArguments") -> Optional[str]:
    if not args.conductor_port:
        return None
    return f"http://{args.conductor_host}:{args.conductor_port}"


def _vision_base(args: "ListenAndPlayRealtimeArguments") -> Optional[str]:
    if not args.vision_port:
        return None
    return f"http://{args.vision_host}:{args.vision_port}"


def _vision_look(base: str, question: str = "", cam: str = "") -> dict:
    """Blocking GET to the vision service's /look (run via asyncio.to_thread).
    VLM inference can take a few seconds (CPU fallback), so allow a long timeout."""
    import urllib.error
    import urllib.parse
    import urllib.request

    params = {}
    if question:
        params["q"] = question
    if cam:
        params["cam"] = cam
    query = f"?{urllib.parse.urlencode(params)}" if params else ""
    req = urllib.request.Request(f"{base}/look{query}", method="GET")
    try:
        with urllib.request.urlopen(req, timeout=30.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            return json.loads(exc.read().decode("utf-8"))
        except Exception:
            return {"ok": False, "error": f"HTTP {exc.code}"}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _look_tool_spec() -> dict:
    return {
        "type": "function",
        "name": "look",
        "description": (
            "Look through the robot's camera and describe what it currently sees. "
            "Call this whenever the user asks what you see, what's in front of you, "
            "to find or count something, or any question about the physical scene. "
            "Pass a specific 'question' to ask about the scene; omit it for a general "
            "description. Returns text you should read back to the user."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "question": {
                    "type": "string",
                    "description": "Optional specific question about the scene, e.g. 'is anyone holding a mug?'",
                }
            },
        },
    }


def _conductor_call(base: str, path: str, payload: Optional[dict] = None) -> dict:
    """Blocking HTTP to the conductor (run via asyncio.to_thread). stdlib only."""
    import urllib.error
    import urllib.request

    url = f"{base}{path}"
    data = None
    method = "GET"
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        method = "POST"
    req = urllib.request.Request(
        url, data=data, method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            return json.loads(exc.read().decode("utf-8"))
        except Exception:
            return {"error": f"HTTP {exc.code}"}
    except Exception as exc:
        return {"error": str(exc)}


def _tool_specs(modes: list[str]) -> list[dict]:
    return [
        {
            "type": "function",
            "name": "set_mode",
            "description": (
                "Switch the robot's behavior mode. 'idle' stops and is safe; "
                "'teleop' lets a remote operator drive the arms and base; "
                "'local_teleop' lets the local joystick drive the base."
            ),
            "parameters": {
                "type": "object",
                "properties": {"mode": {"type": "string", "enum": modes}},
                "required": ["mode"],
            },
        },
        {
            "type": "function",
            "name": "move_arms",
            "description": (
                "Move both arms to a named posture, then return to whatever the "
                "robot was doing. 'home' is the resting pose, 'middle' centers all "
                "joints, 'arms_up' raises them."
            ),
            "parameters": {
                "type": "object",
                "properties": {"preset": {"type": "string", "enum": PRESETS}},
                "required": ["preset"],
            },
        },
    ]


def _make_client(args: ListenAndPlayRealtimeArguments) -> AsyncOpenAI:
    base_url = args.base_url or f"http://{args.host}:{args.port}/v1"
    websocket_base_url = args.websocket_base_url or f"ws://{args.host}:{args.port}/v1"
    return AsyncOpenAI(
        api_key=args.api_key,
        base_url=base_url,
        websocket_base_url=websocket_base_url,
    )


def _build_session_update(
    args: ListenAndPlayRealtimeArguments,
    tools: Optional[list[dict]] = None,
) -> dict:
    def maybe_pcm_format(rate: int) -> Optional[dict]:
        # The OpenAI realtime Pydantic models only validate audio/pcm at 24 kHz.
        # Our local pipeline defaults to 16 kHz internally when format is omitted,
        # so omit the field for the common local case instead of sending an
        # invalid 16 kHz declaration.
        if rate == 16000:
            return None
        if rate == 24000:
            return {"type": "audio/pcm", "rate": 24000}
        raise ValueError(
            f"Unsupported rate {rate}. Use 16000 for the local pipeline default "
            f"or 24000 to match the OpenAI realtime audio format schema."
        )

    input_cfg = {
        "turn_detection": {"type": "server_vad", "interrupt_response": True},
    }
    output_cfg: dict[str, Any] = {}

    input_format = maybe_pcm_format(args.send_rate)
    output_format = maybe_pcm_format(args.recv_rate)
    if input_format is not None:
        input_cfg["format"] = input_format
    if output_format is not None:
        output_cfg["format"] = output_format
    if args.voice:
        output_cfg["voice"] = args.voice

    session = {
        "type": "realtime",
        "audio": {
            "input": input_cfg,
            "output": output_cfg,
        },
    }
    if args.instructions:
        session["instructions"] = args.instructions
    if tools:
        session["tools"] = tools
        session["tool_choice"] = "auto"
    return {"type": "session.update", "session": session}


async def listen_and_play_realtime(
    args: ListenAndPlayRealtimeArguments,
    external_stop: Optional[Event] = None,
) -> None:
    import sounddevice as sd
    import websockets

    client = _make_client(args)

    conductor_base = _conductor_base(args)
    vision_base = _vision_base(args)

    mic_queue: Queue[bytes] = Queue(maxsize=128)
    # When embedded in another process (e.g. viser_control forks this on a
    # worker thread), the caller passes its own Event to request shutdown.
    # Standalone CLI use gets a fresh Event stopped via stdin (see below).
    stop_event = external_stop if external_stop is not None else Event()
    playback_buffer = bytearray()
    playback_lock = Lock()
    speaker_active_until = [0.0]
    partial_user_text = ""
    live_user_width = 0
    saw_user_speech = False
    # This server emits tool calls *inside* an open response (the model speaks its
    # acknowledgement in the same turn), so a follow-up response.create must wait
    # until that response finishes — else "conversation_already_has_active_response".
    response_active = [False]
    pending_followup = [False]

    loop = asyncio.get_running_loop()
    face_clients: set = set()
    face_state = "idle"
    face_amp = [0.0]
    face_amp_lock = Lock()
    last_user_transcript = [""]
    last_assistant_transcript = [""]
    assistant_transcript_buf = [""]

    def broadcast_face(payload: dict) -> None:
        if not face_clients:
            return
        data = json.dumps(payload)
        for ws in list(face_clients):
            asyncio.create_task(_safe_send(ws, data))

    async def _safe_send(ws, data: str) -> None:
        try:
            await ws.send(data)
        except Exception:
            face_clients.discard(ws)

    def set_face_state(new_state: str) -> None:
        nonlocal face_state
        if new_state == face_state:
            return
        face_state = new_state
        broadcast_face({"t": "state", "v": new_state})

    async def face_handler(ws):
        face_clients.add(ws)
        try:
            await ws.send(json.dumps({"t": "state", "v": face_state}))
            if last_user_transcript[0]:
                await ws.send(json.dumps({
                    "t": "transcript", "role": "user",
                    "text": last_user_transcript[0], "final": True,
                }))
            if last_assistant_transcript[0]:
                await ws.send(json.dumps({
                    "t": "transcript", "role": "assistant",
                    "text": last_assistant_transcript[0], "final": True,
                }))
            async for _ in ws:
                pass
        except Exception:
            pass
        finally:
            face_clients.discard(ws)

    async def face_pump():
        interval = 1.0 / max(1, args.face_rate)
        silent_since: Optional[float] = None
        while not stop_event.is_set():
            await asyncio.sleep(interval)
            with face_amp_lock:
                amp = face_amp[0]
            with playback_lock:
                buf_empty = len(playback_buffer) == 0
            now = time.monotonic()
            if buf_empty and amp < 0.01:
                silent_since = silent_since or now
                if face_state == "speaking" and now - silent_since > 0.25:
                    set_face_state("idle")
            else:
                silent_since = None
            broadcast_face({"t": "mouth", "v": round(amp, 3)})

    def render_live_user_text(text: str, final: bool = False) -> None:
        nonlocal live_user_width
        line = f"USER: {text}"
        padded = line
        if live_user_width > len(line):
            padded += " " * (live_user_width - len(line))

        if final:
            print(f"\r{padded}", flush=True)
            live_user_width = 0
            return

        print(f"\r{padded}", end="", flush=True)
        live_user_width = len(line)

    def clear_live_user_text() -> None:
        nonlocal live_user_width
        if live_user_width == 0:
            return
        print("\r" + (" " * live_user_width) + "\r", end="", flush=True)
        live_user_width = 0

    def clear_playback_buffer() -> None:
        speaker_active_until[0] = 0.0
        with playback_lock:
            playback_buffer.clear()

    def callback_recv(outdata, _frames, _time_info, status):
        if status:
            print(f"Speaker status: {status}", flush=True)

        needed = len(outdata)
        with playback_lock:
            available = min(needed, len(playback_buffer))
            if available:
                outdata[:available] = playback_buffer[:available]
                del playback_buffer[:available]
            if available < needed:
                outdata[available:] = b"\x00" * (needed - available)

        if available:
            samples = np.frombuffer(bytes(outdata[:available]), dtype=np.int16).astype(np.float32)
            rms = float(np.sqrt(np.mean(samples * samples))) / 32768.0
            amp = min(1.0, rms * 4.0)
        else:
            amp = 0.0
        with face_amp_lock:
            face_amp[0] = amp * 0.5 + face_amp[0] * 0.5

    def callback_send(indata, _frames, _time_info, status):
        if status:
            print(f"Mic status: {status}", flush=True)

        if args.block_mic_during_playback:
            with playback_lock:
                speaker_active = bool(playback_buffer)
            if speaker_active or time.monotonic() < speaker_active_until[0]:
                return

        try:
            mic_queue.put_nowait(bytes(indata))
        except Exception:
            pass

    def run_tool(name: str, arguments: str) -> dict:
        """Execute one tool call against the conductor. Blocking; call via
        asyncio.to_thread. Returns a small dict fed back to the model."""
        try:
            call_args = json.loads(arguments) if arguments else {}
        except Exception:
            return {"ok": False, "error": "could not parse tool arguments"}

        if name == "look":
            if vision_base is None:
                return {"ok": False, "error": "vision is offline"}
            question = (call_args.get("question") or "").strip()
            res = _vision_look(vision_base, question)
            if res.get("ok"):
                return {"ok": True, "description": res.get("text", "")}
            return {"ok": False, "error": res.get("error", "look failed")}

        if conductor_base is None:
            return {"ok": False, "error": "robot control is offline"}

        if name == "set_mode":
            mode = call_args.get("mode")
            res = _conductor_call(conductor_base, "/mission", {"name": mode})
            if res.get("ok"):
                return {"ok": True, "mode": res.get("mission", mode)}
            return {"ok": False, "error": res.get("error", "mode switch failed")}

        if name == "move_arms":
            preset = call_args.get("preset")
            res = _conductor_call(conductor_base, "/preset", {"preset": preset})
            if res.get("ok"):
                return {"ok": True, "preset": preset, "resuming": res.get("resumes")}
            return {"ok": False, "error": res.get("error", "arm move failed")}

        return {"ok": False, "error": f"unknown tool {name}"}

    async def handle_tool_call(conn, name: str, call_id: str, arguments: str) -> None:
        result = await asyncio.to_thread(run_tool, name, arguments)
        print(f"TOOL RESULT: {name} -> {result}", flush=True)
        await conn.send({
            "type": "conversation.item.create",
            "item": {
                "type": "function_call_output",
                "call_id": call_id,
                "output": json.dumps(result),
            },
        })
        # Write tools (set_mode/move_arms): the model already acknowledged inline
        # in the same response, so a second response.create would double-speak AND
        # collide with the still-open response — only follow up to report a FAILURE.
        # Read tools (look): the result IS the payload the model must speak, so
        # always follow up. Either way, defer until the active response closes
        # (flushed in the response.done handler) to avoid the collision.
        needs_followup = (name in READ_TOOLS) or (not result.get("ok", False))
        if needs_followup:
            if response_active[0]:
                pending_followup[0] = True
            else:
                await conn.send({"type": "response.create"})

    async def send_audio(conn):
        while not stop_event.is_set():
            try:
                chunk = await asyncio.to_thread(mic_queue.get, True, 0.1)
            except Empty:
                continue

            await conn.send(
                {
                    "type": "input_audio_buffer.append",
                    "audio": base64.b64encode(chunk).decode("ascii"),
                }
            )

    async def receive_events(conn):
        nonlocal partial_user_text, saw_user_speech

        while not stop_event.is_set():
            event = await conn.recv()

            if args.print_json:
                try:
                    print(f"EVENT: {event.model_dump_json()}", flush=True)
                except Exception:
                    print(f"EVENT: {event}", flush=True)

            if event.type == "session.created":
                print("Connected.", flush=True)
                set_face_state("idle")
            elif event.type == "input_audio_buffer.speech_started":
                clear_playback_buffer()
                partial_user_text = ""
                if saw_user_speech:
                    print("", flush=True)
                saw_user_speech = True
                set_face_state("listening")
            elif event.type == "input_audio_buffer.speech_stopped":
                set_face_state("thinking")
            elif event.type == "conversation.item.input_audio_transcription.delta":
                # This server currently sends the latest partial hypothesis in
                # each "delta" event rather than a token-level suffix, so render
                # the newest snapshot instead of concatenating repeated text.
                partial_user_text = event.delta.strip()
                if partial_user_text:
                    render_live_user_text(partial_user_text)
                    broadcast_face({
                        "t": "transcript", "role": "user",
                        "text": partial_user_text, "final": False,
                    })
            elif event.type == "conversation.item.input_audio_transcription.completed":
                partial_user_text = ""
                final_user = event.transcript.strip()
                render_live_user_text(final_user, final=True)
                last_user_transcript[0] = final_user
                broadcast_face({
                    "t": "transcript", "role": "user",
                    "text": final_user, "final": True,
                })
            elif event.type == "response.created":
                clear_live_user_text()
                assistant_transcript_buf[0] = ""
                response_active[0] = True
                print("ASSISTANT: <response started>", flush=True)
            elif event.type == "response.output_audio.delta":
                audio = base64.b64decode(event.delta)
                with playback_lock:
                    playback_buffer.extend(audio)
                speaker_active_until[0] = time.monotonic() + max(0.15, len(audio) / (2 * args.recv_rate))
                set_face_state("speaking")
            elif event.type == "response.output_audio.done":
                print("ASSISTANT: <audio done>", flush=True)
            elif event.type == "response.output_audio_transcript.delta":
                assistant_transcript_buf[0] += event.delta
                broadcast_face({
                    "t": "transcript", "role": "assistant",
                    "text": assistant_transcript_buf[0], "final": False,
                })
            elif event.type == "response.output_audio_transcript.done":
                print(f"ASSISTANT: {event.transcript}", flush=True)
                last_assistant_transcript[0] = event.transcript
                broadcast_face({
                    "t": "transcript", "role": "assistant",
                    "text": event.transcript, "final": True,
                })
            elif event.type == "response.function_call_arguments.done":
                print(
                    f"TOOL: {event.name} call_id={event.call_id} arguments={event.arguments}",
                    flush=True,
                )
                await handle_tool_call(conn, event.name, event.call_id, event.arguments)
            elif event.type == "response.done":
                if event.response.status == "cancelled":
                    clear_playback_buffer()
                    set_face_state("idle")
                response_active[0] = False
                print(f"ASSISTANT: <response {event.response.status}>", flush=True)
                # A tool failure arrived mid-response; now that it's closed, let
                # the model speak to it.
                if pending_followup[0]:
                    pending_followup[0] = False
                    await conn.send({"type": "response.create"})
            elif event.type == "error":
                clear_live_user_text()
                print(f"ERROR: {event.error.type}: {event.error.message}", flush=True)
            else:
                clear_live_user_text()
                print(f"EVENT: {event.type}", flush=True)

    async def wait_for_stop():
        if external_stop is None:
            # Standalone CLI: block on the terminal.
            await asyncio.to_thread(input, "Press Enter to stop...\n")
        else:
            # Threaded/embedded: block until the owning process signals stop.
            await asyncio.to_thread(external_stop.wait)
        stop_event.set()

    input_stream = sd.RawInputStream(
        samplerate=args.send_rate,
        channels=1,
        dtype="int16",
        blocksize=args.chunk_size,
        callback=callback_send,
        device=args.input_device,
    )
    # If opening the speaker fails, release the mic we just opened — otherwise a
    # crashed run leaks the capture device and blocks every retry (and confuses
    # ALSA/PipeWire about who owns the card).
    try:
        output_stream = sd.RawOutputStream(
            samplerate=args.recv_rate,
            channels=1,
            dtype="int16",
            blocksize=args.chunk_size,
            callback=callback_recv,
            device=args.output_device,
        )
    except BaseException:
        input_stream.close()
        raise

    input_stream.start()
    output_stream.start()

    face_server = None
    if args.face_port:
        face_server = await websockets.serve(face_handler, args.face_host, args.face_port)
        print(f"Face WS on ws://{args.face_host}:{args.face_port}", flush=True)

    tools: list[dict] = []
    if conductor_base is not None:
        modes = DEFAULT_MODES
        status = await asyncio.to_thread(_conductor_call, conductor_base, "/modes", None)
        if isinstance(status.get("modes"), list) and status["modes"]:
            modes = status["modes"]
        else:
            print(f"Conductor {conductor_base} unreachable; tools use default modes.", flush=True)
        tools += _tool_specs(modes)
        print(f"Robot tools enabled ({conductor_base}); modes={modes}", flush=True)
    if vision_base is not None:
        tools.append(_look_tool_spec())
        print(f"Vision 'look' tool enabled ({vision_base}).", flush=True)
    tools = tools or None

    try:
        async with client.realtime.connect(model=args.model) as conn:
            await conn.send(_build_session_update(args, tools))  # type: ignore[arg-type]

            sender_task = asyncio.create_task(send_audio(conn))
            receiver_task = asyncio.create_task(receive_events(conn))
            stopper_task = asyncio.create_task(wait_for_stop())
            pump_task = asyncio.create_task(face_pump())

            done, pending = await asyncio.wait(
                {sender_task, receiver_task, stopper_task, pump_task},
                return_when=asyncio.FIRST_COMPLETED,
            )

            stop_event.set()
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)

            for task in done:
                exc = task.exception()
                if exc is not None:
                    raise exc
    finally:
        stop_event.set()
        clear_live_user_text()
        input_stream.stop()
        output_stream.stop()
        input_stream.close()
        output_stream.close()
        if face_server is not None:
            face_server.close()
            await face_server.wait_closed()


def main() -> None:
    parser = argparse.ArgumentParser(description="Talk to the local OpenAI-compatible realtime speech pipeline.")
    defaults = ListenAndPlayRealtimeArguments()
    parser.add_argument("--host", default=defaults.host)
    parser.add_argument("--port", type=int, default=defaults.port)
    parser.add_argument("--model", default=defaults.model)
    parser.add_argument("--api-key", default=defaults.api_key)
    parser.add_argument("--base-url", default=defaults.base_url)
    parser.add_argument("--websocket-base-url", default=defaults.websocket_base_url)
    parser.add_argument("--send-rate", type=int, default=defaults.send_rate)
    parser.add_argument("--recv-rate", type=int, default=defaults.recv_rate)
    parser.add_argument("--chunk-size", type=int, default=defaults.chunk_size)
    parser.add_argument("--input-device", type=int, default=defaults.input_device)
    parser.add_argument("--output-device", type=int, default=defaults.output_device)
    parser.add_argument("--instructions", default=defaults.instructions)
    parser.add_argument(
        "--voice",
        default=defaults.voice,
        help=("session.audio.output.voice (Kokoro id like bm_fable, or OpenAI name like marin)."),
    )
    parser.add_argument("--print-json", action="store_true", default=defaults.print_json)
    parser.add_argument(
        "--block-mic-during-playback",
        action="store_true",
        default=defaults.block_mic_during_playback,
    )
    parser.add_argument("--face-host", default=defaults.face_host)
    parser.add_argument("--face-port", type=int, default=defaults.face_port,
                        help="Face WebSocket port. Set to 0 to disable.")
    parser.add_argument("--face-rate", type=int, default=defaults.face_rate)
    parser.add_argument("--conductor-host", default=defaults.conductor_host,
                        help="Conductor behavior-tree API host (mode/preset tools).")
    parser.add_argument("--conductor-port", type=int, default=defaults.conductor_port,
                        help="Conductor API port. Set to 0 to disable robot tools.")
    parser.add_argument("--vision-host", default=defaults.vision_host,
                        help="Vision scene-description service host (the 'look' tool).")
    parser.add_argument("--vision-port", type=int, default=defaults.vision_port,
                        help="Vision service port. Set to 0 to disable the look tool.")
    namespace = parser.parse_args()
    args = ListenAndPlayRealtimeArguments(
        host=namespace.host,
        port=namespace.port,
        model=namespace.model,
        api_key=namespace.api_key,
        base_url=namespace.base_url,
        websocket_base_url=namespace.websocket_base_url,
        send_rate=namespace.send_rate,
        recv_rate=namespace.recv_rate,
        chunk_size=namespace.chunk_size,
        input_device=namespace.input_device,
        output_device=namespace.output_device,
        instructions=namespace.instructions,
        voice=namespace.voice,
        print_json=namespace.print_json,
        block_mic_during_playback=namespace.block_mic_during_playback,
        face_host=namespace.face_host,
        face_port=namespace.face_port,
        face_rate=namespace.face_rate,
        conductor_host=namespace.conductor_host,
        conductor_port=namespace.conductor_port,
        vision_host=namespace.vision_host,
        vision_port=namespace.vision_port,
    )
    try:
        asyncio.run(listen_and_play_realtime(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
