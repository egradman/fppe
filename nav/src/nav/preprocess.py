"""ROS-free image preprocessing for the visual-nav models.

`transform_images` is copied from visualnav-transformer/deployment/src/utils.py
(the canonical reference) with the ROS `sensor_msgs` dependency removed. The
model was trained against exactly this transform, so it must match bit-for-bit.
"""

from typing import List

import torch
from PIL import Image as PILImage
from torchvision import transforms
import torchvision.transforms.functional as TF

# ViNT/GNM training aspect ratio (width / height) used for center-cropping.
IMAGE_ASPECT_RATIO = 4 / 3

_NORM = transforms.Compose(
    [
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ]
)


def transform_images(
    pil_imgs, image_size: List[int], center_crop: bool = False
) -> torch.Tensor:
    """Transform a list of PIL images into a single channel-stacked tensor.

    Returns shape [1, 3*N, H, W] for N input images (context frames stacked on
    the channel dim), matching what ViNT.forward expects for `obs_img`.
    """
    if not isinstance(pil_imgs, list):
        pil_imgs = [pil_imgs]
    transf_imgs = []
    for pil_img in pil_imgs:
        w, h = pil_img.size
        if center_crop:
            if w > h:
                pil_img = TF.center_crop(pil_img, (h, int(h * IMAGE_ASPECT_RATIO)))
            else:
                pil_img = TF.center_crop(pil_img, (int(w / IMAGE_ASPECT_RATIO), w))
        pil_img = pil_img.resize(image_size)  # PIL resize takes (width, height)
        transf_img = _NORM(pil_img).unsqueeze(0)
        transf_imgs.append(transf_img)
    return torch.cat(transf_imgs, dim=1)


def load_frames_from_video(path: str, n_context: int, stride: int, goal_ahead: int):
    """Pull a context window + a goal frame from a video file.

    Returns (context_frames: List[PILImage], goal_frame: PILImage).
    context_frames has length n_context (oldest->newest); goal_frame is
    `goal_ahead` frames beyond the newest context frame — a stand-in for "a
    place a bit further along the route" for a sanity check.
    """
    import cv2

    cap = cv2.VideoCapture(path)
    frames = []
    want = n_context * stride + goal_ahead + 1
    idx = 0
    while len(frames) < want:
        ret, bgr = cap.read()
        if not ret:
            break
        if idx % 1 == 0:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            frames.append(PILImage.fromarray(rgb))
        idx += 1
    cap.release()
    if len(frames) < n_context + 1:
        raise RuntimeError(
            f"video {path} too short: got {len(frames)} frames, need >= {n_context + 1}"
        )
    ctx_idxs = [i * stride for i in range(n_context)]
    ctx_idxs = [min(i, len(frames) - 1) for i in ctx_idxs]
    goal_idx = min(ctx_idxs[-1] + goal_ahead, len(frames) - 1)
    context = [frames[i] for i in ctx_idxs]
    goal = frames[goal_idx]
    return context, goal
