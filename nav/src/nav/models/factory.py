"""Model construction + checkpoint loading (no third-party wrapper).

The official Berkeley checkpoints store `checkpoint["model"]` as a *pickled model
object* whose class lives under the training package `vint_train.models.*`. We
vendored that code under `nav.models.*`, so unpickling needs the old module path
remapped `vint_train` -> `nav`. `_RemapUnpickler` does exactly that.

Weights come from the official Pre-Trained Models Google Drive folder linked in
the visualnav-transformer README.
"""

import pickle
from pathlib import Path

import torch

from nav.models.vint.vint import ViNT

# Official "Pre-Trained Models" folder from the visualnav-transformer README.
OFFICIAL_DRIVE_FOLDER = "1a9yWR2iooXFAqjQHetz263--4_2FFggg"
WEIGHTS_FILENAME = {"vint": "vint.pth", "gnm": "gnm_large.pth", "nomad": "nomad.pth"}


def get_model(config: dict) -> torch.nn.Module:
    """Construct an (untrained) model matching the config."""
    mt = config["model_type"]
    if mt == "vint":
        return ViNT(
            context_size=config["context_size"],
            len_traj_pred=config["len_traj_pred"],
            learn_angle=config["learn_angle"],
            obs_encoder=config["obs_encoder"],
            obs_encoding_size=config["obs_encoding_size"],
            late_fusion=config["late_fusion"],
            mha_num_attention_heads=config["mha_num_attention_heads"],
            mha_num_attention_layers=config["mha_num_attention_layers"],
            mha_ff_dim_factor=config["mha_ff_dim_factor"],
        )
    raise NotImplementedError(f"model_type={mt!r} not vendored yet (Phase 1 is vint-only)")


class _Ignored:
    """Placeholder for training-only objects (optimizer/scheduler) we don't load.

    The checkpoint is a full training dict; we only read `["model"]`, but pickle
    still resolves every class in the dict. Training-only deps (e.g.
    `warmup_scheduler`) aren't installed here, so we substitute this inert stub.
    """

    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        pass


class _RemapUnpickler(pickle.Unpickler):
    """Remap the checkpoint's stale training-package module paths to our vendor,
    and stub out classes from modules that aren't installed (training-only)."""

    def find_class(self, module, name):
        if module == "vint_train" or module.startswith("vint_train."):
            module = "nav" + module[len("vint_train"):]
        try:
            return super().find_class(module, name)
        except (ModuleNotFoundError, AttributeError):
            return _Ignored


class _RemapPickle:
    Unpickler = _RemapUnpickler


def load_weights(config: dict, model: torch.nn.Module, ckpt_path, device: str) -> torch.nn.Module:
    """Load a .pth checkpoint into `model` (tolerant of minor key mismatches)."""
    checkpoint = torch.load(
        ckpt_path, map_location=device, pickle_module=_RemapPickle, weights_only=False
    )
    loaded = checkpoint["model"]
    try:
        state_dict = loaded.module.state_dict()  # DataParallel-wrapped
    except AttributeError:
        state_dict = loaded.state_dict()
    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    if missing or unexpected:
        print(f"[nav] load_state_dict: {len(missing)} missing, {len(unexpected)} unexpected keys")
    return model


def ensure_weights(model_type: str, weights_dir: Path) -> Path:
    """Return the local checkpoint path, downloading the official folder if absent."""
    weights_dir.mkdir(parents=True, exist_ok=True)
    fname = WEIGHTS_FILENAME[model_type]
    target = weights_dir / fname
    if target.exists():
        return target
    hits = list(weights_dir.rglob(fname))
    if hits:
        return hits[0]

    import gdown

    print(f"[nav] downloading official pretrained weights -> {weights_dir}")
    gdown.download_folder(
        id=OFFICIAL_DRIVE_FOLDER, output=str(weights_dir), quiet=False, use_cookies=False
    )
    hits = list(weights_dir.rglob(fname))
    if not hits:
        raise FileNotFoundError(
            f"{fname} not found under {weights_dir} after download; "
            f"check the official folder contents."
        )
    return hits[0]
