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
}
MODEL_ID = os.environ.get("QWEN_MODEL", PRESETS["4b"])

_TOOL_CALL = re.compile(r"<tool_call>\s*(.*?)\s*</tool_call>", re.S)
_THINK = re.compile(r"<think>.*?</think>", re.S)

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
        _model = load(MODEL_ID)
    return _model


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
                continue
            calls.append({"name": call.get("name", ""), "arguments": call.get("arguments") or {}})
        text_out = _TOOL_CALL.sub("", raw).strip()

        self.messages.append({"role": "assistant", "content": text_out, "tool_calls": [
            {"type": "function", "function": c} for c in calls]})
        return {"type": "call" if calls else "text", "function_calls": calls,
                "reasoning": text_out, "confidence": None}

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
