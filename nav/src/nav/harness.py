"""Phase 1 harness: load a policy, run it on an image sequence, print outputs.

This does NOT touch the robot. It exists to prove the model loads on this GPU
and emits sane (distance, waypoint) tensors, so Phase 2 (the executor) can build
on a known-good inference call.

Example:
    uv run nav-harness --media ../scratch/test.mp4
    uv run nav-harness --media frames/ --device cuda
"""

import argparse
import sys

import numpy as np
import torch

from .model import load_config, load_model
from .preprocess import load_frames_from_video, transform_images


def _num_action_params(config: dict) -> int:
    return 2 + (2 if config.get("learn_angle", True) else 0)


def run_vint(model, config, context_frames, goal_frame, device):
    img_size = config["image_size"]
    obs_img = transform_images(context_frames, img_size, center_crop=False)
    goal_img = transform_images(goal_frame, img_size, center_crop=False)
    obs_img = obs_img.to(device)
    goal_img = goal_img.to(device)

    with torch.no_grad():
        dist_pred, action_pred = model(obs_img, goal_img)

    dist = float(dist_pred.flatten()[0].cpu())
    nap = _num_action_params(config)
    action = action_pred.reshape(-1, config["len_traj_pred"], nap)[0].cpu().numpy()
    return dist, action, obs_img.shape, goal_img.shape


def main():
    p = argparse.ArgumentParser(description="Phase 1 visual-nav inference harness")
    p.add_argument("--media", required=True, help="video file (mp4) to sample frames from")
    p.add_argument("--config", default="vint.yaml")
    p.add_argument("--device", default="auto", help="cuda | cpu | auto")
    p.add_argument("--stride", type=int, default=3, help="frames between context samples")
    p.add_argument("--goal-ahead", type=int, default=30, help="frames from newest ctx to goal")
    args = p.parse_args()

    device = args.device
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"

    config = load_config(args.config)
    print(f"[nav] model_type={config['model_type']}  device={device}")
    print(f"[nav] torch={torch.__version__}  cuda_available={torch.cuda.is_available()}")
    if device == "cuda":
        print(f"[nav] gpu={torch.cuda.get_device_name(0)}")

    context_frames, goal_frame = load_frames_from_video(
        args.media, config["context_size"] + 1, args.stride, args.goal_ahead
    )
    print(f"[nav] loaded {len(context_frames)} context frames + 1 goal frame")

    print("[nav] loading model + weights (first run downloads checkpoint via gdown)...")
    model = load_model(config, device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"[nav] model loaded: {n_params/1e6:.1f}M params")

    if config["model_type"] != "vint":
        print(f"[nav] harness currently drives model_type=vint only; got {config['model_type']}")
        sys.exit(2)

    dist, action, obs_shape, goal_shape = run_vint(
        model, config, context_frames, goal_frame, device
    )

    print()
    print(f"[nav] obs_img shape:  {tuple(obs_shape)}  (expect [1, 3*(ctx+1), H, W])")
    print(f"[nav] goal_img shape: {tuple(goal_shape)}")
    print(f"[nav] predicted temporal distance to goal: {dist:.3f}")
    print(f"[nav] raw action deltas [len_traj_pred x action_params]:")
    print(np.array2string(action, precision=3, suppress_small=True))
    # Integrate the (dx, dy) deltas into a cumulative path for a readability check:
    xy = np.cumsum(action[:, :2], axis=0)
    print("[nav] cumulative (x, y) waypoints:")
    print(np.array2string(xy, precision=3, suppress_small=True))
    fwd = xy[-1, 0]
    print(
        f"[nav] net forward (x) displacement over horizon: {fwd:.3f} "
        f"({'forward' if fwd > 0 else 'backward/neutral'})"
    )
    print("[nav] OK — forward pass produced finite outputs:",
          bool(np.isfinite(dist) and np.isfinite(action).all()))


if __name__ == "__main__":
    main()
