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


def _make_client(args: ListenAndPlayRealtimeArguments) -> AsyncOpenAI:
    base_url = args.base_url or f"http://{args.host}:{args.port}/v1"
    websocket_base_url = args.websocket_base_url or f"ws://{args.host}:{args.port}/v1"
    return AsyncOpenAI(
        api_key=args.api_key,
        base_url=base_url,
        websocket_base_url=websocket_base_url,
    )


def _build_session_update(args: ListenAndPlayRealtimeArguments) -> dict:
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
    return {"type": "session.update", "session": session}


async def listen_and_play_realtime(
    args: ListenAndPlayRealtimeArguments,
    external_stop: Optional[Event] = None,
) -> None:
    import sounddevice as sd
    import websockets

    client = _make_client(args)

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
            elif event.type == "response.done":
                if event.response.status == "cancelled":
                    clear_playback_buffer()
                    set_face_state("idle")
                print(f"ASSISTANT: <response {event.response.status}>", flush=True)
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
    output_stream = sd.RawOutputStream(
        samplerate=args.recv_rate,
        channels=1,
        dtype="int16",
        blocksize=args.chunk_size,
        callback=callback_recv,
        device=args.output_device,
    )

    input_stream.start()
    output_stream.start()

    face_server = None
    if args.face_port:
        face_server = await websockets.serve(face_handler, args.face_host, args.face_port)
        print(f"Face WS on ws://{args.face_host}:{args.face_port}", flush=True)

    try:
        async with client.realtime.connect(model=args.model) as conn:
            await conn.send(_build_session_update(args))  # type: ignore[arg-type]

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
    )
    try:
        asyncio.run(listen_and_play_realtime(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
