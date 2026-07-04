"""`vision serve` — run the scene-description HTTP service on skynet.

  just vision serve                 # bind 0.0.0.0:8102, cameras via fppe:8091
  just vision serve --preload       # load Moondream at startup (else first /look)
  just vision look "what do you see" # one-shot local query for testing
"""
from __future__ import annotations

import argparse
import logging

from vision.model import REVISION, Moondream


def main() -> None:
    parser = argparse.ArgumentParser(description="fppe scene-description service (Moondream2).")
    sub = parser.add_subparsers(dest="cmd")

    serve = sub.add_parser("serve", help="run the HTTP service")
    serve.add_argument("--host", default="0.0.0.0")
    serve.add_argument("--port", type=int, default=8102)
    serve.add_argument("--pi-base", default="http://fppe:8091", help="viser_control base URL for camera streams")
    serve.add_argument("--cam", default="cam0", help="default camera label")
    serve.add_argument("--revision", default=REVISION, help="Moondream HF revision to pin")
    serve.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"],
                       help="auto = CUDA only if it has headroom, else CPU (skynet GPU is shared)")
    serve.add_argument("--preload", action="store_true", help="load the model at startup, not on first request")

    look = sub.add_parser("look", help="one-shot: grab a frame and describe it (test path)")
    look.add_argument("question", nargs="?", default="", help="optional VQA question")
    look.add_argument("--pi-base", default="http://fppe:8091")
    look.add_argument("--cam", default="cam0")
    look.add_argument("--revision", default=REVISION)
    look.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("pyvips").setLevel(logging.WARNING)  # silence VIPS threadpool chatter

    if args.cmd == "look":
        from vision.frame import grab_frame

        model = Moondream(revision=args.revision, device=args.device)
        image = grab_frame(f"{args.pi_base.rstrip('/')}/mjpeg/{args.cam}")
        print(model.describe(image, args.question))
        return

    # default: serve
    if args.cmd != "serve":
        parser.print_help()
        return

    from vision.server import make_server

    model = Moondream(revision=args.revision, device=args.device)
    if args.preload:
        model.ensure()
    httpd = make_server(model, host=args.host, port=args.port, pi_base=args.pi_base, cam=args.cam)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
