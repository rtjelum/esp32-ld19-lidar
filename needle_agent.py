"""
needle_agent.py — Needle agent that loads its tools from needle_tools.json.

Tool schemas and their call tree both live in needle_tools.json: each tool's
"x-exec" block says what to run for each call.  Needle receives the schemas
with "x-exec" stripped, and execute() below interprets the call tree.
"""

import ast
import importlib.util
import json
import os
import re
import subprocess
import sys
import textwrap
import time

import qwen_agent

try:
    import needle
except ImportError:
    print("Please install needle: pip install cactus-needle")
    raise SystemExit(1)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

TOOLS_FILE = os.path.join(os.path.dirname(__file__), "needle_tools.json")
RECORDINGS_DIR = "recordings"
CHAT_SCRIPTS_DIR = "chat_scripts"
SCRIPT_TIMEOUT = 300  # seconds before /run gives up on a script
MAX_ROUNDS = 8

# Needle returns a calibrated 0-1 confidence with each response. Below this
# threshold we don't auto-execute the tool calls — we ask the user to confirm
# first. (Confidence is None for fine-tuned models, in which case we skip the
# gate.) Override with the NEEDLE_CONFIDENCE_MIN env var.
CONFIDENCE_MIN = float(os.environ.get("NEEDLE_CONFIDENCE_MIN", "0.5"))

# "auto": Needle first, falling back to Qwen (qwen_agent.py) when Needle errors
# or is below CONFIDENCE_MIN on its first step.  "needle" / "qwen": one only.
# Switch at runtime with /model.
BACKENDS = ("auto", "needle", "qwen")
BACKEND = os.environ.get("AGENT_BACKEND", "auto")


def load_tools(path: str = TOOLS_FILE):
    """Return (schemas_json_for_needle, {tool_name: x-exec spec}, tool_count)."""
    with open(path) as f:
        tools = json.load(f)
    exec_specs = {t["function"]["name"]: t["x-exec"] for t in tools if "x-exec" in t}
    schemas = [{k: v for k, v in t.items() if k != "x-exec"} for t in tools]
    return json.dumps(schemas), exec_specs, len(tools)


def list_recordings() -> list[str]:
    if not os.path.isdir(RECORDINGS_DIR):
        return []
    return sorted(f for f in os.listdir(RECORDINGS_DIR) if f.endswith(".ldim"))


def fmt_call(name: str, args: dict) -> str:
    return f"{name}({', '.join(f'{k}={v!r}' for k, v in args.items())})"

# ---------------------------------------------------------------------------
# Executor — interprets the "x-exec" call tree attached to each tool in
# needle_tools.json.  Keys:
#   run      argv list; "{arg}" placeholders are filled from the call args
#   mode     "background" (detach), "capture" (run, return output) or
#            "stream" (run with output to the terminal, return pass/fail)
#   timeout  seconds for capture/stream
#   status   message returned on success
#   open     file to open with `open` after a successful capture
#   switch   arg whose value selects an entry in "cases" (merged over
#            "defaults"); values not in "cases" are rejected
#   resolve  {arg: resolver} — rewrite an arg before dispatch (e.g. "ldim")
# ---------------------------------------------------------------------------

def resolve_ldim(name: str):
    """Map a scan base name to recordings/<name>.ldim (fuzzy fallback)."""
    base = name[:-5] if name.endswith(".ldim") else name
    target = f"{RECORDINGS_DIR}/{base}.ldim"
    if os.path.exists(target):
        return target, None
    files = list_recordings()
    matches = [f for f in files if base in f]
    if matches:
        return f"{RECORDINGS_DIR}/{matches[0]}", None
    return None, {"error": f"{target} not found in {RECORDINGS_DIR}/.",
                  "available_recordings": [f[:-5] for f in files]}

RESOLVERS = {"ldim": resolve_ldim}


def spawn(argv):
    subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def execute(spec: dict, args: dict) -> dict:
    ctx = dict(args)
    for arg, kind in spec.get("resolve", {}).items():
        value, err = RESOLVERS[kind](ctx.get(arg, ""))
        if err:
            return err
        ctx[arg] = value
        ctx["stem"] = os.path.splitext(value)[0]

    if "switch" in spec:
        key, cases = spec["switch"], spec["cases"]
        if ctx.get(key) not in cases:
            return {"error": f"Invalid {key} '{ctx.get(key)}'. Must be one of: {list(cases)}"}
        step = {**spec.get("defaults", {}), **cases[ctx[key]]}
    else:
        step = spec

    argv = [a.format(**ctx) for a in step["run"]]
    cmdline = " ".join(argv)
    mode = step.get("mode", "capture")
    timeout = step.get("timeout", 120)

    if mode == "background":
        spawn(argv)
        return {"status": step.get("status", f"Launched `{cmdline}`.").format(**ctx)}

    if mode == "stream":
        # Output goes straight to the terminal; the model only gets pass/fail so
        # it can't be derailed by verbose build output.
        print(f"  [exec] running `{cmdline}` (output below)...")
        res = subprocess.run(argv, timeout=timeout)
        if res.returncode == 0:
            return {"status": step.get("status", "Done.").format(**ctx)}
        return {"error": f"`{cmdline}` failed (exit {res.returncode}); see terminal output above."}

    res = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    if res.returncode == 0 and "open" in step:
        ctx["open"] = step["open"].format(**ctx)
        if os.path.exists(ctx["open"]):
            spawn(["open", ctx["open"]])
            return {"status": step.get("status", "Opened {open}.").format(**ctx)}
    return {"stdout": res.stdout, "stderr": res.stderr, "returncode": res.returncode}

# ---------------------------------------------------------------------------
# Agent — single prompt turn with tool-execution loop
# ---------------------------------------------------------------------------

class LowConfidence(Exception):
    pass


def confirm_calls(header: str, response: dict, calls: list) -> bool:
    """Failsafe: show calls we're unsure about and ask the user to run them."""
    print(f"\n  [!] {header} for these tool call(s):")
    for call in calls:
        print(f"        {fmt_call(call.get('name', ''), call.get('arguments') or {})}")
    if response.get("reasoning"):
        print(f"      reasoning: {response['reasoning']}")
    try:
        ans = input("  Execute these? [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = ""
    return ans in ("y", "yes")


def run(prompt: str, agent, exec_specs: dict, fallback=False, confirm_first=False) -> dict:
    """Send prompt to the agent, execute any tool calls, return the final response.

    fallback:      raise LowConfidence instead of asking, if the first step is
                   below CONFIDENCE_MIN (nothing has been executed yet).
    confirm_first: always ask before executing the first step's calls.
    """
    response = agent.complete(prompt)

    # Small models sometimes get stuck re-emitting the same (already-failed)
    # call. Cache results by (name, args) so a repeat isn't re-executed, and
    # break the loop once a round produces nothing but repeats.
    seen = {}
    results = []

    for round_no in range(MAX_ROUNDS):
        calls = response.get("function_calls") or []
        if response.get("type") != "call" or not calls:
            break

        conf = response.get("confidence")
        low = conf is not None and conf < CONFIDENCE_MIN
        if low and fallback and round_no == 0:
            raise LowConfidence(conf)
        if low or (confirm_first and round_no == 0):
            header = f"Low confidence ({conf:.0%})" if low else "Fallback model proposes"
            if not confirm_calls(header, response, calls):
                print("  Skipped.")
                return {"type": "abort", "reasoning": "Skipped: user declined.",
                        "confidence": conf, "results": []}

        results = []
        all_repeats = True
        for call in calls:
            name = call.get("name", "")
            args = call.get("arguments") or {}
            print(f"  [tool] {fmt_call(name, args)}")
            key = (name, json.dumps(args, sort_keys=True))
            if key not in seen:
                all_repeats = False
                spec = exec_specs.get(name)
                try:
                    seen[key] = execute(spec, args) if spec else {"error": f"unknown tool: {name}"}
                except Exception as exc:
                    seen[key] = {"error": str(exc)}
            results.append(seen[key])

        if all_repeats:  # nothing new this round — the model is looping
            break
        response = agent.complete(json.dumps(results))

    response["results"] = results
    return response

# ---------------------------------------------------------------------------
# Slash commands
# ---------------------------------------------------------------------------

def cmd_help(_ctx, _arg):
    print("\nAvailable slash commands:")
    for name, (desc, _) in COMMANDS.items():
        print(f"  {name:<16} {desc}")
    print("\nAnything else is sent to Needle as a natural-language prompt.")


def cmd_tools(ctx, _arg):
    print(f"\n{ctx['count']} tools loaded from {TOOLS_FILE}:")
    for t in json.loads(ctx["schemas"]):
        fn = t.get("function", t)
        print(f"  {fn['name']:<28} {fn.get('description', '')}")


def cmd_recordings(_ctx, _arg):
    files = list_recordings()
    if not files:
        print(f"No .ldim files found in {RECORDINGS_DIR}/.")
        return
    print(f"\n{len(files)} recording(s) in {RECORDINGS_DIR}/:")
    for f in files:
        size_kb = os.path.getsize(os.path.join(RECORDINGS_DIR, f)) // 1024
        print(f"  {f}  ({size_kb} KB)")


def cmd_model(ctx, arg):
    if arg:
        arg = arg.lower()
        if arg not in BACKENDS:
            print(f"Unknown backend '{arg}'. Choose one of: {', '.join(BACKENDS)}")
            return
        if arg != "needle" and not ctx["qwen_ok"]:
            print("Qwen needs mlx-lm: .venv/bin/pip install mlx-lm")
            return
        ctx["backend"] = arg
    print(f"Backend: {ctx['backend']}  (fallback model: {qwen_agent.MODEL_ID})")


CHAT_SYSTEM = ("The user is on macOS (Apple Silicon) with Python 3. Any Python script you "
               "write must run on macOS: no Windows-only modules such as msvcrt, winreg or "
               "winsound (use curses, or termios/tty, for key presses).")


def chat_turn(ctx, text: str):
    if not ctx["chat_history"]:
        ctx["chat_history"].append({"role": "system", "content": CHAT_SYSTEM})
    ctx["chat_history"].append({"role": "user", "content": text})
    try:
        reply = qwen_agent.chat(ctx["chat_history"], prefix="Qwen> ")
    except BaseException:
        ctx["chat_history"].pop()  # don't keep a turn that never got a reply
        raise
    ctx["chat_history"].append({"role": "assistant", "content": reply})
    offer_save_scripts(ctx, reply)


# Any fenced block; the closing fence is optional so a reply cut off mid-script
# is still caught.  Tag-less blocks count only if they look like Python.
_CODE_BLOCK = re.compile(r"^[ \t]*```[ \t]*([\w+-]*)[^\n]*\n(.*?)(?:^[ \t]*```|\Z)", re.S | re.M)
_PY_TAGS = ("python", "python3", "py")
_PY_HINT = re.compile(r"^\s*(import |from \w+ import |def |class |print\(|if __name__)", re.M)


def python_blocks(reply: str) -> list[str]:
    blocks = []
    for m in _CODE_BLOCK.finditer(reply):
        tag, code = m.group(1).lower(), m.group(2)
        heading = next((l for l in reversed(reply[:m.start()].splitlines()) if l.strip()), "")
        if "output" in heading.lower():  # "Example output:" fenced as python
            continue
        if tag in _PY_TAGS or (not tag and _PY_HINT.search(code)):
            blocks.append(textwrap.dedent(code))
    return blocks


def next_script_path() -> str:
    n = 1
    while os.path.exists(path := os.path.join(CHAT_SCRIPTS_DIR, f"chat_{n}.py")):
        n += 1
    return path


def parses(code: str) -> bool:
    try:
        compile(code, "<chat>", "exec")
        return True
    except SyntaxError:
        return False


def script_from_reply(reply: str):
    """Join every Python block in the reply into one script.

    Models often split a script (imports in one block, the rest in the next),
    so all blocks are kept, in order.  Blocks that aren't valid Python (e.g.
    "Example output" fenced as python) are dropped — unless none parse, which
    happens when the reply was cut off mid-script; then keep everything.
    Returns (code, n_blocks_used, n_blocks_dropped).
    """
    blocks = [b.strip("\n") for b in python_blocks(reply)]
    blocks = [b for b in blocks if b.strip()]
    good = [b for b in blocks if parses(b)] or blocks
    return "\n\n".join(good) + "\n" if good else "", len(good), len(blocks) - len(good)


def log_reply(reply: str):
    """Keep the raw reply so a bad extraction can be compared against it."""
    os.makedirs(CHAT_SCRIPTS_DIR, exist_ok=True)
    with open(os.path.join(CHAT_SCRIPTS_DIR, "chat_log.md"), "a") as f:
        f.write(f"\n\n---- {time.strftime('%Y-%m-%d %H:%M:%S')}\n{reply}\n")


def offer_save_scripts(ctx, reply: str):
    """If the chat reply contains Python code, offer to save it as one script."""
    code, used, dropped = script_from_reply(reply)
    if not code:
        return
    log_reply(reply)
    lines = code.splitlines()
    first = next((l for l in lines if l.strip()), "")
    print(f"  [save] Python script: {len(lines)} lines from {used} code block(s), "
          f"starts with: {first.strip()[:60]}")
    if dropped:
        print(f"  [save] skipped {dropped} block(s) that aren't valid Python (example output?)")
    if not parses(code):
        print("  [save] warning: the script doesn't parse — it may be incomplete.")
    default = next_script_path()
    try:
        ans = input(f"  Save as [{default}]? Enter/y = yes, n = skip, or type a filename: ").strip()
    except (EOFError, KeyboardInterrupt):
        ans = "n"
    if ans.lower() in ("n", "no"):
        return
    path = default if ans.lower() in ("", "y", "yes") else ans
    if not path.endswith(".py"):
        path += ".py"
    if os.path.dirname(path) == "":
        path = os.path.join(CHAT_SCRIPTS_DIR, path)
    if os.path.exists(path) and not ask_yes(f"  {path} exists. Overwrite? [y/N] "):
        print("  Skipped.")
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write(code)
    print(f"  Saved {path}")
    ctx["last_script"] = path
    if ask_yes(f"  Run {path} now? [y/N] "):
        run_script(path)


def ask_yes(question: str) -> bool:
    try:
        return input(question).strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        return False


# Import name -> pip package, where they differ.
PIP_NAMES = {"PIL": "pillow", "cv2": "opencv-python", "sklearn": "scikit-learn",
             "skimage": "scikit-image", "yaml": "pyyaml", "bs4": "beautifulsoup4",
             "serial": "pyserial", "dateutil": "python-dateutil",
             "pygame": "pygame-ce"}  # classic pygame has no Python 3.14 wheels


def missing_modules(path: str) -> list[str]:
    """Top-level imports in the script that this venv can't resolve."""
    try:
        with open(path) as f:
            tree = ast.parse(f.read())
    except (SyntaxError, OSError):
        return []  # let the run itself report it
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    here = os.path.dirname(os.path.abspath(path))  # sibling modules the script can import
    return sorted(n for n in names
                  if importlib.util.find_spec(n) is None
                  and not os.path.exists(os.path.join(here, n + ".py"))
                  and not os.path.isdir(os.path.join(here, n)))


def install_missing(path: str) -> bool:
    """Offer to pip-install a script's missing imports. False = don't run it."""
    missing = missing_modules(path)
    if not missing:
        return True
    # Standard-library modules that don't exist here (msvcrt, winreg, winsound
    # are Windows-only) can't come from pip — the script needs rewriting.
    other_os = [m for m in missing if m in sys.stdlib_module_names]
    pkgs = [PIP_NAMES.get(m, m) for m in missing if m not in other_os]
    print(f"  [run] missing module(s): {', '.join(missing)}")
    if other_os:
        print(f"  [run] {', '.join(other_os)}: part of Python on another OS (e.g. Windows), "
              "not installable here.\n        Ask Qwen for a macOS version of the script.")
    if pkgs:
        if not ask_yes(f"  pip install {' '.join(pkgs)} into this venv? [y/N] "):
            return ask_yes("  Run anyway? [y/N] ")
        res = subprocess.run([sys.executable, "-m", "pip", "install", *pkgs])
        importlib.invalidate_caches()
        if res.returncode != 0:
            print("  [run] pip install failed; see output above.")
            return ask_yes("  Run anyway? [y/N] ")
    return not other_os or ask_yes("  Run anyway? [y/N] ")


def run_script(path: str):
    """Run a script with this venv's Python; output goes straight to the terminal."""
    if not install_missing(path):
        print("  Not run.")
        return
    print(f"  [run] {os.path.basename(sys.executable)} {path}  (Ctrl-C to stop)", flush=True)
    proc = subprocess.Popen([sys.executable, path])
    try:
        code = proc.wait(timeout=SCRIPT_TIMEOUT)
    except (KeyboardInterrupt, subprocess.TimeoutExpired) as exc:
        proc.terminate()
        proc.wait()
        why = "interrupted" if isinstance(exc, KeyboardInterrupt) else f"timed out after {SCRIPT_TIMEOUT}s"
        print(f"\n  [run] {why}; stopped.")
        return
    print(f"  [run] exit code {code}")


def cmd_run(ctx, arg):
    path = arg or ctx.get("last_script")
    if not path:
        print("Usage: /run <script.py>  (defaults to the last script saved from chat)")
        return
    if not os.path.exists(path) and os.path.exists(os.path.join(CHAT_SCRIPTS_DIR, path)):
        path = os.path.join(CHAT_SCRIPTS_DIR, path)
    if not os.path.exists(path):
        print(f"{path} not found.")
        return
    run_script(path)


def cmd_chat(ctx, arg):
    if not ctx["qwen_ok"]:
        print("Chat needs mlx-lm: .venv/bin/pip install mlx-lm")
        return
    if arg:  # one-off message; history is shared with chat mode
        chat_turn(ctx, arg)
        return
    ctx["chat_mode"] = not ctx["chat_mode"]
    if ctx["chat_mode"]:
        print(f"Chat mode on ({qwen_agent.MODEL_ID}, no tools). /chat again to leave.")
    else:
        ctx["chat_history"].clear()
        print("Chat mode off; back to tool agent.")


def cmd_qwen(_ctx, arg):
    if arg:
        print(f"Qwen model: {qwen_agent.set_model(arg)} (loads on next use)")
        return
    print(f"Qwen model: {qwen_agent.MODEL_ID}\nPresets (/qwen <name>, or any MLX repo id):")
    for name, repo in qwen_agent.PRESETS.items():
        print(f"  {name:<6} {repo}")


COMMANDS = {
    "/help":       ("Show this help message.", cmd_help),
    "/tools":      ("List tools loaded from needle_tools.json.", cmd_tools),
    "/recordings": ("List .ldim files in the local recordings/ folder.", cmd_recordings),
    "/model":      ("Show or set the backend: /model auto|needle|qwen.", cmd_model),
    "/qwen":       ("Show or switch the Qwen model: /qwen 0.6b|1.7b|4b|8b|14b.", cmd_qwen),
    "/chat":       ("Toggle plain chat with Qwen (no tools), or /chat <message>.", cmd_chat),
    "/run":        ("Run a Python script: /run [file] (default: last saved from chat).", cmd_run),
    "/clear":      ("Clear the terminal screen.", lambda _ctx, _arg: os.system("clear")),
    "/quit":       ("Exit.", None),
}


def setup_readline():
    """Arrow-key history and slash-command tab completion."""
    try:
        import readline
    except ImportError:
        return

    def completer(text, state):
        options = [cmd for cmd in COMMANDS if cmd.startswith(text)]
        return options[state] if state < len(options) else None

    readline.set_completer(completer)
    if "libedit" in (readline.__doc__ or "").lower():
        readline.parse_and_bind("bind ^I rl_complete")
    else:
        readline.parse_and_bind("tab: complete")
    # Keep '/' out of the word delimiters so '/h' completes as one word.
    readline.set_completer_delims(readline.get_completer_delims().replace("/", ""))

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def run_with(make_agent, prompt: str, ctx: dict, **kw) -> dict:
    # Fresh agent per prompt so no context bleeds between turns.
    agent = make_agent(tools=ctx["schemas"])
    try:
        return run(prompt, agent, ctx["exec_specs"], **kw)
    finally:
        agent.close()


def ask(prompt: str, ctx: dict):
    backend = ctx["backend"]
    result = None
    if backend != "qwen":
        can_fall_back = backend == "auto" and ctx["qwen_ok"]
        try:
            result = run_with(needle.Needle, prompt, ctx, fallback=can_fall_back)
        except LowConfidence as low:
            print(f"  [!] Needle low confidence ({low.args[0]:.0%}); trying {qwen_agent.MODEL_ID}...")
        except Exception as exc:
            if not can_fall_back:
                raise
            print(f"  [!] Needle failed ({exc}); trying {qwen_agent.MODEL_ID}...")
    if result is None:
        result = run_with(qwen_agent.QwenAgent, prompt, ctx, confirm_first=backend == "auto")
    print(f"\n[Reason]:  {result.get('reasoning', '')}")
    results = result.get("results") or []
    if results:
        shown = results[0] if len(results) == 1 else results
        print(f"[Result]:  {json.dumps(shown, indent=2)}")


def main():
    schemas, exec_specs, count = load_tools()
    qwen_ok = qwen_agent.available()
    backend = BACKEND if BACKEND in BACKENDS else "auto"
    if backend == "qwen" and not qwen_ok:
        backend = "needle"
    ctx = {"schemas": schemas, "exec_specs": exec_specs, "count": count,
           "backend": backend, "qwen_ok": qwen_ok,
           "chat_mode": False, "chat_history": [], "last_script": None}
    setup_readline()
    print(f"Loading Needle with {count} tools from {os.path.basename(TOOLS_FILE)}...")
    print(f"Backend: {backend}" + ("" if qwen_ok else "  (Qwen fallback off: mlx-lm not installed)"))
    print("Ready. Type a prompt or /help for commands.")

    while True:
        try:
            print()
            prompt = input("Chat> " if ctx["chat_mode"] else "User> ").strip()
            if not prompt:
                continue
            cmd, _, arg = prompt.partition(" ")
            cmd = cmd.lower()
            if cmd == "/quit" or prompt.lower() in ("exit", "quit"):
                break
            if cmd in COMMANDS:
                COMMANDS[cmd][1](ctx, arg.strip())
            elif ctx["chat_mode"]:
                chat_turn(ctx, prompt)
            else:
                ask(prompt, ctx)
        except (KeyboardInterrupt, EOFError):
            break
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()
