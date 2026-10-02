"""
qwen_agent.py — Qwen3 (via mlx-lm) behind the same interface as needle.Needle.

complete(text) returns a Needle-shaped dict:
    {"type": "call" | "text", "function_calls": [{"name", "arguments"}],
     "reasoning": str, "confidence": None}
The first complete() is the user prompt; later ones are the JSON list of tool
results that needle_agent.run() feeds back.
"""

import json
import os
import re

# 4-bit MLX builds: much smaller and faster than the full-precision repos.
# Sizes are the download; RAM use is a bit more.  Switch at runtime with /qwen.
PRESETS = {
    "0.6b": "Qwen/Qwen3-0.6B",                # 1.5 GB, fast but weak
    "1.7b": "mlx-community/Qwen3-1.7B-4bit",  # 1.0 GB
    "4b":   "mlx-community/Qwen3-4B-4bit",    # 2.3 GB, good default on 16 GB
    "8b":   "mlx-community/Qwen3-8B-4bit",    # 4.6 GB, better code, ~half the speed
    "14b":  "mlx-community/Qwen3-14B-4bit",   # 8.3 GB, tight on a 16 GB Mac
    # Ternary Qwen3.8-27B in Hadamard-rotated 2-bit; loaded by _load_prism below.
    "bonsai-27b": "prism-ml/Ternary-Bonsai-2-27B-mlx-2bit",  # 8.6 GB (7.7 GB text), tight on 16 GB
}
MODEL_ID = os.environ.get("QWEN_MODEL", PRESETS["bonsai-27b"])

_TOOL_CALL = re.compile(r"<tool_call>\s*(.*?)\s*</tool_call>", re.S)
_THINK = re.compile(r"<think>.*?</think>", re.S)
# MiniCPM5 style: <function name="f"><param name="p">value</param></function>
_XML_CALL = re.compile(r'<function name="([^"]+)">(.*?)</function>', re.S)
# Spark-X2.5 style: <tool_call>f<arg_key>p</arg_key><arg_value>value</arg_value></tool_call>
_ARG_PAIR = re.compile(r"<arg_key>(.*?)</arg_key>\s*<arg_value>(.*?)</arg_value>", re.S)
_XML_PARAM = re.compile(r'<param name="([^"]+)">(?:<!\[CDATA\[(.*?)\]\]>|(.*?))</param>', re.S)

# Loaded once per process; the first load downloads the model from Hugging Face.
_model = None


def set_model(name: str) -> str:
    """Switch to a preset ("8b") or any MLX repo id; loads lazily on next use."""
    global MODEL_ID, _model
    MODEL_ID = PRESETS.get(name.lower(), name)
    _model = None
    return MODEL_ID


def available() -> bool:
    try:
        import mlx_lm  # noqa: F401
        return True
    except ImportError:
        return False


def _load():
    global _model
    if _model is None:
        from mlx_lm import load
        print(f"  [qwen] loading {MODEL_ID} (first use)...")
        if _model_type(MODEL_ID) == "prism_hadamard_qwen35":
            _model = _load_prism(MODEL_ID)
        else:
            # Some repos ship transformers modeling code; mlx-lm has its own
            # implementation, so never run (or prompt to run) remote code.
            _model = load(MODEL_ID, tokenizer_config={"trust_remote_code": False})
    return _model


def _model_type(repo: str) -> str | None:
    """config.json's model_type, fetching only that file (cached by the hub)."""
    try:
        if os.path.isdir(repo):
            path = os.path.join(repo, "config.json")
        else:
            from huggingface_hub import hf_hub_download
            path = hf_hub_download(repo, "config.json")
        with open(path) as f:
            return json.load(f).get("model_type")
    except Exception:
        return None  # let mlx_lm.load report the real problem


# --- Prism ML "Bonsai" packs ---------------------------------------------------
# Ternary weights stored as MLX affine 2-bit in a Hadamard-rotated basis, so
# each packed layer has to rotate its input first (the embedding rotates its
# output back).  Stock mlx_lm.load can't do that.  This mirrors the pack's own
# runtime/runtime.py (Packed) and runtime/vision_artifact.py, text-only so the
# 0.9 GB vision tower and mlx-vlm aren't needed, and kept here rather than
# imported from the download so no repo code is executed.

def _fwht(x, block, signs, inverse=False):
    import math
    import mlx.core as mx
    shape, dtype = x.shape, x.dtype
    x = x.astype(mx.float32)
    if not inverse:
        x = x * signs
    x = mx.hadamard_transform(x.reshape(-1, block), scale=1 / math.sqrt(block)).reshape(shape)
    if inverse:
        x = x * signs
    return x.astype(dtype)


def _packed_class():
    import mlx.core as mx
    from mlx import nn

    class Packed(nn.Module):
        def __init__(self, weight, scales, biases, block, signs, embedding):
            super().__init__()
            self.weight, self.scales, self.biases = weight, scales, biases
            self.block, self.signs, self.embedding = block, signs, embedding

        def __call__(self, x):
            if self.embedding:
                idx = x.reshape(-1)
                out = mx.dequantize(self.weight[idx], self.scales[idx], self.biases[idx],
                                    group_size=128, bits=2)
                out = out.reshape(*x.shape, -1).astype(mx.float16)
                return _fwht(out, self.block, self.signs, inverse=True) if self.block else out
            if self.block:
                x = _fwht(x, self.block, self.signs)
            return mx.quantized_matmul(x, self.weight, self.scales, self.biases,
                                       transpose=True, group_size=128, bits=2)

    return Packed


def _load_prism(repo: str):
    import mlx.core as mx
    from huggingface_hub import snapshot_download
    from mlx_lm.models.qwen3_5 import TextModel, TextModelArgs
    from mlx_lm.tokenizer_utils import load as load_tokenizer

    path = repo if os.path.isdir(repo) else snapshot_download(
        repo, allow_patterns=["*.json", "*.jinja", "model.safetensors"])
    with open(os.path.join(path, "config.json")) as f:
        config = json.load(f)
    if config.get("base_model_type") != "qwen3_5":
        raise ValueError(f"unsupported Prism base model {config.get('base_model_type')}")
    model = TextModel(TextModelArgs.from_dict(config["text_config"]))
    # Tensors use mlx-vlm names ("language_model.model.layers...", "vision_tower...");
    # keep the language model only.  mx.load is lazy, so the tower is never read.
    prefix = "language_model."
    weights = {k[len(prefix):]: v for k, v in mx.load(os.path.join(path, "model.safetensors")).items()
               if k.startswith(prefix)}
    Packed = _packed_class()
    for rec in config["modules"]:
        name, block = rec["path"], rec["block"]
        if rec["dtype"] != "float16" or (block and block not in (512, 1024, 2048, 4096)):
            raise ValueError(f"unsupported packed module {name}")
        signs = weights.get(name + ".signs")
        if block and signs is None:
            raise ValueError(f"missing Hadamard signs for {name}")
        *parents, attr = name.split(".")
        parent = model
        for part in parents:
            parent = parent[int(part)] if part.isdigit() else getattr(parent, part)
        setattr(parent, attr, Packed(*(weights[f"{name}.{s}"] for s in ("weight", "scales", "biases")),
                                     block, signs, rec["embedding"]))
    model.load_weights(list(model.sanitize(weights).items()), strict=True)
    model.eval()
    mx.eval(model.parameters())

    eos = None
    gen_path = os.path.join(path, "generation_config.json")
    if os.path.exists(gen_path):
        with open(gen_path) as f:
            eos = json.load(f).get("eos_token_id")
        eos = eos if isinstance(eos, list) or eos is None else [eos]
    from pathlib import Path
    tokenizer = load_tokenizer(Path(path), {"trust_remote_code": False}, eos_token_ids=eos)
    return model, tokenizer


class QwenAgent:
    def __init__(self, tools: str, system: str | None = None):
        self.tools = json.loads(tools) if isinstance(tools, str) else tools
        self.messages = [{"role": "system", "content": system}] if system else []
        self.model, self.tokenizer = _load()

    def complete(self, text: str, max_new_tokens: int = 512) -> dict:
        if any(m["role"] == "user" for m in self.messages):
            for result in json.loads(text):
                self.messages.append({"role": "tool", "content": json.dumps(result)})
        else:
            self.messages.append({"role": "user", "content": text})

        from mlx_lm import generate
        prompt = self.tokenizer.apply_chat_template(
            self.messages, tools=self.tools, add_generation_prompt=True,
            tokenize=False, enable_thinking=False)
        raw = _THINK.sub("", generate(self.model, self.tokenizer, prompt,
                                      max_tokens=max_new_tokens))

        calls = []
        for block in _TOOL_CALL.findall(raw):
            try:
                call = json.loads(block)
            except json.JSONDecodeError:
                name = block.split("<arg_key>", 1)[0].strip()
                if name:
                    calls.append({"name": name, "arguments": {
                        k.strip(): self._coerce(name, k.strip(), v)
                        for k, v in _ARG_PAIR.findall(block)}})
                continue
            calls.append({"name": call.get("name", ""), "arguments": call.get("arguments") or {}})
        for name, body in _XML_CALL.findall(raw):
            args = {k: self._coerce(name, k, cdata or plain)
                    for k, cdata, plain in _XML_PARAM.findall(body)}
            calls.append({"name": name, "arguments": args})
        text_out = _XML_CALL.sub("", _TOOL_CALL.sub("", raw)).strip()

        self.messages.append({"role": "assistant", "content": text_out, "tool_calls": [
            {"type": "function", "function": c} for c in calls]})
        return {"type": "call" if calls else "text", "function_calls": calls,
                "reasoning": text_out, "confidence": None}

    def _coerce(self, tool: str, param: str, value: str):
        """XML params arrive as text; JSON-decode the ones the schema says aren't strings."""
        for t in self.tools:
            fn = t.get("function", t)
            if fn.get("name") == tool:
                kind = fn.get("parameters", {}).get("properties", {}).get(param, {}).get("type")
                if kind and kind != "string":
                    try:
                        return json.loads(value)
                    except json.JSONDecodeError:
                        pass
        return value

    def close(self):
        pass


def chat(messages: list, prefix: str = "", max_new_tokens: int = 4096) -> str:
    """Plain multi-turn chat, no tools.  Streams the reply to stdout and returns it.

    messages is the running [{"role", "content"}] history; the caller appends
    both the user turn and the returned assistant reply.
    """
    from mlx_lm import stream_generate
    model, tokenizer = _load()
    prompt = tokenizer.apply_chat_template(
        messages, add_generation_prompt=True, tokenize=False, enable_thinking=False)
    print(prefix, end="", flush=True)
    parts = []
    for chunk in stream_generate(model, tokenizer, prompt, max_tokens=max_new_tokens):
        print(chunk.text, end="", flush=True)
        parts.append(chunk.text)
    print()
    if chunk.finish_reason == "length":
        print(f"  [qwen] reply cut off at {max_new_tokens} tokens.")
    return _THINK.sub("", "".join(parts)).strip()
