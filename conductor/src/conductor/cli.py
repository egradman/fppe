"""conductor CLI.

    conductor run <mode>     # run the ticking engine against a live fppe Pi
    conductor list           # list available modes
    conductor show <mode>    # render a mode's tree (no hardware)
    conductor catalog        # print the LLM-facing behavior catalog (JSON)
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import threading

import py_trees

from conductor.base_cmd import BaseCmdChannel
from conductor.client import FakeRobotClient, RobotClient
from conductor.context import Context
from conductor.dsl import build_tree
from conductor.engine import Engine
from conductor.modes import list_modes, load_mode
from conductor.nav_client import NavClient
from conductor.perception import PerceptionClient
from conductor.registry import catalog
from conductor.senders import FakeSenderManager, SenderManager
from conductor.server import make_server


def _fake_ctx(fppe_host: str = "fppe") -> Context:
    return Context(client=FakeRobotClient(), senders=FakeSenderManager(fail_unavailable=False),
                   fppe_host=fppe_host)


def cmd_list(_args) -> int:
    for m in list_modes():
        print(m)
    return 0


def cmd_catalog(_args) -> int:
    print(json.dumps(catalog(), indent=2))
    return 0


def cmd_show(args) -> int:
    dsl = load_mode(args.mode)
    tree = build_tree(dsl, _fake_ctx())
    print(py_trees.display.unicode_tree(tree, show_status=False))
    return 0


def _real_ctx(host: str, api_port: int, nav_url: str = "http://localhost:8107") -> Context:
    return Context(
        client=RobotClient(host=host, api_port=api_port),
        senders=SenderManager(fppe_host=host),
        fppe_host=host,
        base=BaseCmdChannel(host=host),
        # No detector wired yet: perception reports "not seen", so chase_dogs
        # degrades to an endless search. Attach detector=/frame_source= here.
        perception=PerceptionClient(),
        # nav_to_goal reads its command from nav-serve (default :8107). If the
        # service is down, NavToGoal reports offline and the mission fails to idle.
        nav=NavClient(base_url=nav_url),
    )


def cmd_run(args) -> int:
    dsl = load_mode(args.mode)
    ctx = _real_ctx(args.host, args.api_port, args.nav_url)
    engine = Engine(ctx, tick_hz=args.tick_hz)
    engine.setup()
    engine.load_mission(args.mode, dsl)

    last = {"mission": None}

    def on_tick(eng: Engine) -> None:
        cur = eng.current_mission()
        if cur != last["mission"]:
            last["mission"] = cur
            print(f"[mission] {cur}")
            print(py_trees.display.unicode_tree(eng.root, show_status=True))

    print(f"conductor: running {args.mode!r} against {args.host}:{args.api_port} "
          f"@ {args.tick_hz} Hz (Ctrl-C to stop -> idle)")
    try:
        engine.spin(on_tick=on_tick)
    except KeyboardInterrupt:
        print("\nconductor: shutting down -> idle + stopping senders")
    finally:
        engine.shutdown()
    return 0


def cmd_serve(args) -> int:
    ctx = _real_ctx(args.host, args.api_port, args.nav_url)
    engine = Engine(ctx, tick_hz=args.tick_hz)
    engine.setup()
    engine.load_mission(args.mode, load_mode(args.mode))  # boot mode

    httpd = make_server(engine, host=args.bind, port=args.port)
    spinner = threading.Thread(target=engine.spin, name="engine-spin", daemon=True)
    spinner.start()

    print(f"conductor: serving on {args.bind}:{args.port}, engine @ {args.tick_hz} Hz "
          f"against {args.host}:{args.api_port}; boot mode {args.mode!r} (Ctrl-C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nconductor: stopping -> idle + teardown")
    finally:
        httpd.shutdown()
        engine.shutdown()
        spinner.join(timeout=2.0)
    return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser(prog="conductor", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="list available modes")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("catalog", help="print the behavior catalog as JSON")
    sp.set_defaults(func=cmd_catalog)

    sp = sub.add_parser("show", help="render a mode's tree (no hardware)")
    sp.add_argument("mode")
    sp.set_defaults(func=cmd_show)

    sp = sub.add_parser("run", help="run the engine against a live fppe Pi")
    sp.add_argument("mode")
    sp.add_argument("--host", default="fppe", help="fppe hostname (default: fppe)")
    sp.add_argument("--api-port", type=int, default=8091)
    sp.add_argument("--nav-url", default="http://localhost:8107", help="nav-serve base URL")
    sp.add_argument("--tick-hz", type=float, default=10.0)
    sp.set_defaults(func=cmd_run)

    sp = sub.add_parser("serve", help="run the engine + HTTP mode API (browser/LLM control)")
    sp.add_argument("--mode", default="idle", help="boot mode (default: idle)")
    sp.add_argument("--host", default="fppe", help="fppe hostname (default: fppe)")
    sp.add_argument("--api-port", type=int, default=8091, help="fppe viser_control port")
    sp.add_argument("--nav-url", default="http://localhost:8107", help="nav-serve base URL")
    sp.add_argument("--bind", default="0.0.0.0", help="conductor API bind address")
    sp.add_argument("--port", type=int, default=8100, help="conductor API port (default: 8100)")
    sp.add_argument("--tick-hz", type=float, default=10.0)
    sp.set_defaults(func=cmd_serve)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
