"""
needle_agent.py — Needle agent that loads its tools from needle_tools.json.

Tool schemas and their call tree both live in needle_tools.json: each tool's
"x-exec" block says what to run for each call.  Needle receives the schemas
with "x-exec" stripped, and execute() below interprets the call tree.
"""

import json
import os
import subprocess

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
MAX_ROUNDS = 8

# Needle returns a calibrated 0-1 confidence with each response. Below this
# threshold we don't auto-execute the tool calls — we ask the user to confirm
# first. (Confidence is None for fine-tuned models, in which case we skip the
# gate.) Override with the NEEDLE_CONFIDENCE_MIN env var.
CONFIDENCE_MIN = float(os.environ.get("NEEDLE_CONFIDENCE_MIN", "0.5"))


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

def confirm_low_confidence(response: dict, calls: list) -> bool:
    """Failsafe: show calls the model isn't confident about and ask to run them."""
    print(f"\n  [!] Low confidence ({response['confidence']:.0%}) for these tool call(s):")
    for call in calls:
        print(f"        {fmt_call(call.get('name', ''), call.get('arguments') or {})}")
    if response.get("reasoning"):
        print(f"      reasoning: {response['reasoning']}")
    try:
        ans = input("  Execute these? [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = ""
    return ans in ("y", "yes")


def run(prompt: str, agent, exec_specs: dict) -> dict:
    """Send prompt to Needle, execute any tool calls, return the final response."""
    response = agent.complete(prompt)

    # Small models sometimes get stuck re-emitting the same (already-failed)
    # call. Cache results by (name, args) so a repeat isn't re-executed, and
    # break the loop once a round produces nothing but repeats.
    seen = {}
    results = []

    for _ in range(MAX_ROUNDS):
        calls = response.get("function_calls") or []
        if response.get("type") != "call" or not calls:
            break

        conf = response.get("confidence")
        if conf is not None and conf < CONFIDENCE_MIN and not confirm_low_confidence(response, calls):
            print("  Skipped.")
            return {"type": "abort", "reasoning": "Skipped: low confidence, user declined.",
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

def cmd_help(_ctx):
    print("\nAvailable slash commands:")
    for name, (desc, _) in COMMANDS.items():
        print(f"  {name:<16} {desc}")
    print("\nAnything else is sent to Needle as a natural-language prompt.")


def cmd_tools(ctx):
    print(f"\n{ctx['count']} tools loaded from {TOOLS_FILE}:")
    for t in json.loads(ctx["schemas"]):
        fn = t.get("function", t)
        print(f"  {fn['name']:<28} {fn.get('description', '')}")


def cmd_recordings(_ctx):
    files = list_recordings()
    if not files:
        print(f"No .ldim files found in {RECORDINGS_DIR}/.")
        return
    print(f"\n{len(files)} recording(s) in {RECORDINGS_DIR}/:")
    for f in files:
        size_kb = os.path.getsize(os.path.join(RECORDINGS_DIR, f)) // 1024
        print(f"  {f}  ({size_kb} KB)")


COMMANDS = {
    "/help":       ("Show this help message.", cmd_help),
    "/tools":      ("List tools loaded from needle_tools.json.", cmd_tools),
    "/recordings": ("List .ldim files in the local recordings/ folder.", cmd_recordings),
    "/clear":      ("Clear the terminal screen.", lambda _ctx: os.system("clear")),
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

def ask_needle(prompt: str, ctx: dict):
    # Fresh Needle instance per prompt so no context bleeds between turns.
    agent = needle.Needle(tools=ctx["schemas"])
    try:
        result = run(prompt, agent, ctx["exec_specs"])
    finally:
        agent.close()
    print(f"\n[Reason]:  {result.get('reasoning', '')}")
    results = result.get("results") or []
    if results:
        shown = results[0] if len(results) == 1 else results
        print(f"[Result]:  {json.dumps(shown, indent=2)}")


def main():
    schemas, exec_specs, count = load_tools()
    ctx = {"schemas": schemas, "exec_specs": exec_specs, "count": count}
    setup_readline()
    print(f"Loading Needle with {count} tools from {os.path.basename(TOOLS_FILE)}...")
    print("Ready. Type a prompt or /help for commands.")

    while True:
        try:
            print()
            prompt = input("User> ").strip()
            if not prompt:
                continue
            cmd = prompt.lower().split()[0]
            if cmd == "/quit" or prompt.lower() in ("exit", "quit"):
                break
            if cmd in COMMANDS:
                COMMANDS[cmd][1](ctx)
            else:
                ask_needle(prompt, ctx)
        except (KeyboardInterrupt, EOFError):
            break
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()
