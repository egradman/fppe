"""Model construction + weight loading, using our vendored Berkeley model code.

The model definitions under `nav.models.*` are copied verbatim from the official
visualnav-transformer repo (MIT); see `nav/src/nav/models/__init__.py`. Weights
are the official pretrained checkpoints (see `factory.ensure_weights`).
"""

from pathlib import Path

import yaml

from nav.models.factory import ensure_weights, get_model, load_weights

CONFIG_DIR = Path(__file__).resolve().parents[2] / "configs"
WEIGHTS_DIR = Path(__file__).resolve().parents[2] / "weights"


def load_config(name: str = "vint.yaml") -> dict:
    with open(CONFIG_DIR / name) as f:
        return yaml.safe_load(f)


def load_model(config: dict, device: str):
    """Build the model and load (downloading if needed) its official checkpoint."""
    model = get_model(config)
    ckpt_path = ensure_weights(config["model_type"], WEIGHTS_DIR)
    model = load_weights(config, model, ckpt_path, device)
    model = model.to(device)
    model.eval()
    return model
