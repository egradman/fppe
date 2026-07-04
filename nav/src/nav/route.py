"""Offline route builder (Phase 3): a recorded tour -> an ordered breadcrumb route.

Takes a tour session captured by viser_control's TourRecorder (frames.jsonl +
labels.jsonl + frames/) and subsamples it into an ordered list of *nodes* — the
breadcrumbs the nav-serve follower drives between. For the garage test the route
is a single linear chain (one continuous drive), so the frame order *is* the
graph and no ViNT is needed at build time: spacing is pure dead-reckoning
geometry. ViNT only enters at nav runtime, when the follower decides "am I close
enough to the next breadcrumb to advance." The house map later adds branching +
loop-closure edges on top of this same node format.

Subsampling:
  - keep a new node once translation since the last node >= --spacing OR the
    heading change >= --heading (so turns-in-place still get captured); this
    naturally drops near-duplicate frames while the robot is parked.
  - every *labeled* frame is force-kept as its own node and carries the label —
    so a named goal is exactly the frame the operator tapped at that stop.

Output (portable, rsync-able): <out>/route.json + <out>/nodes/NNN.jpg, with node
image paths stored relative to route.json.

    uv run nav-build-route --session nav/scratch/vivid-maple
"""

import argparse
import json
import math
import shutil
import time
from pathlib import Path


class RouteMap:
    """A built route loaded for navigation: ordered nodes, their goal images, and
    the label -> node-id index. Used by nav-serve's breadcrumb follower.

    Pure metadata by default; call load_images() to pull the node JPEGs (lazy PIL
    import so the builder stays runnable without pillow/torch).
    """

    def __init__(self, path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text())
        self.nodes: list[dict] = self.data["nodes"]
        self.labels: dict[str, list[int]] = self.data.get("labels", {})
        self._images: list | None = None

    def load_images(self):
        from PIL import Image  # lazy: keep route building torch/PIL-free
        imgs = []
        for n in self.nodes:
            im = Image.open(self.path.parent / n["file"]).convert("RGB")
            im.load()
            imgs.append(im)
        self._images = imgs

    def image(self, node_id: int):
        if self._images is None:
            self.load_images()
        return self._images[node_id]

    def resolve_goal(self, label: str | None = None, node: int | None = None, cur: int = 0) -> int:
        """A goal request (label or explicit node id) -> a node id. For a label
        with multiple instances, pick the one nearest `cur` in route order."""
        if node is not None:
            return int(node)
        ids = self.labels.get(label or "")
        if not ids:
            raise KeyError(f"no node labeled {label!r} (have {list(self.labels)})")
        return min(ids, key=lambda i: abs(i - cur))

    def sequence(self, cur: int, goal: int) -> list[int]:
        """Ordered node ids to drive through from `cur` (exclusive) to `goal`
        (inclusive). Linear route, so this is a forward or backward span."""
        if goal == cur:
            return []
        step = 1 if goal > cur else -1
        return list(range(cur + step, goal + step, step))


def _load_session(session: Path) -> tuple[list[dict], list[dict]]:
    frames = [json.loads(l) for l in (session / "frames.jsonl").read_text().splitlines() if l.strip()]
    labels_path = session / "labels.jsonl"
    labels = []
    if labels_path.exists():
        labels = [json.loads(l) for l in labels_path.read_text().splitlines() if l.strip()]
    frames.sort(key=lambda r: r["i"])
    return frames, labels


def build_route(
    session: Path,
    out: Path,
    spacing_m: float = 0.5,
    heading_deg: float = 30.0,
) -> dict:
    frames, labels = _load_session(session)
    if not frames:
        raise SystemExit(f"no frames in {session}")

    # frame index -> list of labels tapped at that frame
    labels_by_frame: dict[int, list[str]] = {}
    for ev in labels:
        labels_by_frame.setdefault(int(ev["frame_i"]), []).append(str(ev["label"]))

    heading_thresh = math.radians(heading_deg)
    by_i = {r["i"]: r for r in frames}

    # Decide which frames become nodes: spacing/heading gate, plus all labeled frames.
    kept: list[int] = []
    last: dict | None = None
    for r in frames:
        forced = r["i"] in labels_by_frame
        if last is None:
            take = True
        else:
            d = math.hypot(r["x"] - last["x"], r["y"] - last["y"])
            dh = abs(r["heading"] - last["heading"])
            take = forced or d >= spacing_m or dh >= heading_thresh
        if take:
            kept.append(r["i"])
            last = r
    # ensure the final frame is a node so the route reaches the true end — but
    # skip it if the robot was already parked there (co-located with the last
    # kept node), which would just make a duplicate end breadcrumb.
    last_kept = by_i[kept[-1]]
    end = frames[-1]
    if end["i"] != kept[-1] and math.hypot(end["x"] - last_kept["x"], end["y"] - last_kept["y"]) >= 0.1:
        kept.append(end["i"])

    # Build node records + copy images.
    nodes_dir = out / "nodes"
    if out.exists():
        shutil.rmtree(out)
    nodes_dir.mkdir(parents=True)

    nodes = []
    label_map: dict[str, list[int]] = {}
    for nid, fi in enumerate(kept):
        r = by_i[fi]
        fname = f"{nid:03d}.jpg"
        shutil.copyfile(session / r["file"], nodes_dir / fname)
        labs = labels_by_frame.get(fi, [])
        for lb in labs:
            label_map.setdefault(lb, []).append(nid)
        nodes.append({
            "id": nid,
            "src_frame": fi,
            "x": r["x"], "y": r["y"], "heading": r["heading"],
            "file": f"nodes/{fname}",
            "labels": labs,
        })

    route = {
        "name": session.name,
        "kind": "linear",  # single continuous tour; order == graph. House adds edges.
        "source": str(session.resolve()),
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "spacing_m": spacing_m,
        "heading_deg": heading_deg,
        "n_frames_in": len(frames),
        "nodes": nodes,
        "labels": label_map,
    }
    (out / "route.json").write_text(json.dumps(route, indent=2))
    return route


def _summarize(route: dict, out: Path):
    print(f"route '{route['name']}' -> {out}/route.json")
    print(f"  {route['n_frames_in']} frames -> {len(route['nodes'])} nodes "
          f"(spacing {route['spacing_m']} m / {route['heading_deg']}°)")
    if route["labels"]:
        for lb, ids in route["labels"].items():
            print(f"  label '{lb}': nodes {ids}")
    else:
        print("  (no labels — route has no named goals)")


def main():
    p = argparse.ArgumentParser(description="Build an ordered breadcrumb route from a recorded tour")
    p.add_argument("--session", required=True, type=Path, help="tour session dir (frames.jsonl, labels.jsonl, frames/)")
    p.add_argument("--out", type=Path, default=None, help="output dir (default: <session>/route)")
    p.add_argument("--spacing", type=float, default=0.5, help="meters of travel between nodes")
    p.add_argument("--heading", type=float, default=30.0, help="degrees of turn that also forces a new node")
    args = p.parse_args()
    out = args.out or (args.session / "route")
    route = build_route(args.session, out, spacing_m=args.spacing, heading_deg=args.heading)
    _summarize(route, out)


if __name__ == "__main__":
    raise SystemExit(main())
