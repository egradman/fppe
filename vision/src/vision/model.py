"""Moondream2 vision-language model wrapper.

A lazy singleton: the (slow, ~a few seconds) weight load happens on first use,
not at import. `describe(image, question)` answers a free-text visual question
(VQA) or, with no question, returns a short caption. Access is serialized with a
lock because the HTTP server is threaded and the GPU model is not re-entrant.

Handles both Moondream API generations:
  - new (2025-* revisions): model.caption(img) / model.query(img, q) -> dicts
  - old (2024-* revisions): model.encode_image(img) + model.answer_question(...)
so pinning a different --revision can't silently break the call path.
"""
from __future__ import annotations

import logging
import threading

log = logging.getLogger(__name__)

MODEL_ID = "vikhyatk/moondream2"
REVISION = "2025-06-21"
DEFAULT_PROMPT = "Describe what you see in one or two sentences."
# skynet's GPU is shared (speech pipeline + games), so `auto` only claims CUDA
# when there's real headroom for Moondream's ~4 GiB fp16 footprint; else CPU.
MIN_CUDA_FREE_BYTES = 4 * 1024**3


class Moondream:
    def __init__(self, model_id: str = MODEL_ID, revision: str = REVISION,
                 device: str = "auto"):
        self.model_id = model_id
        self.revision = revision
        self.requested_device = device
        self._lock = threading.Lock()
        self._model = None
        self._tokenizer = None
        self._device = None

    def _pick_device(self) -> str:
        import torch

        want = self.requested_device
        if want == "cpu" or not torch.cuda.is_available():
            return "cpu"
        if want == "cuda":
            return "cuda"  # explicit: honor even under pressure (may OOM)
        free, _total = torch.cuda.mem_get_info()  # auto
        if free >= MIN_CUDA_FREE_BYTES:
            return "cuda"
        log.info("CUDA only has %.1f GiB free (<%.1f GiB); using CPU",
                 free / 1024**3, MIN_CUDA_FREE_BYTES / 1024**3)
        return "cpu"

    def load(self) -> "Moondream":
        import torch
        from transformers import AutoModelForCausalLM

        device = self._pick_device()
        dtype = torch.float16 if device == "cuda" else torch.float32
        log.info("loading %s@%s on %s (%s)", self.model_id, self.revision, device, dtype)
        model = AutoModelForCausalLM.from_pretrained(
            self.model_id,
            revision=self.revision,
            trust_remote_code=True,
            torch_dtype=dtype,
        ).to(device)
        model.eval()
        tokenizer = None
        if not hasattr(model, "query"):
            from transformers import AutoTokenizer

            tokenizer = AutoTokenizer.from_pretrained(self.model_id, revision=self.revision)
        self._model, self._tokenizer, self._device = model, tokenizer, device
        log.info("moondream ready on %s", device)
        return self

    def ensure(self) -> "Moondream":
        with self._lock:
            if self._model is None:
                self.load()
        return self

    @property
    def loaded(self) -> bool:
        return self._model is not None

    @property
    def device(self) -> str:
        return self._device or "unloaded"

    def describe(self, image, question: str | None = None) -> str:
        """Answer `question` about `image` (a PIL RGB image), or caption it."""
        self.ensure()
        q = (question or "").strip()
        with self._lock:  # serialize GPU access across HTTP worker threads
            if hasattr(self._model, "query"):
                if q:
                    return str(self._model.query(image, q)["answer"]).strip()
                return str(self._model.caption(image, length="short")["caption"]).strip()
            enc = self._model.encode_image(image)
            return str(
                self._model.answer_question(enc, q or DEFAULT_PROMPT, self._tokenizer)
            ).strip()
