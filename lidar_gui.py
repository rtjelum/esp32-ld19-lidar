#!/usr/bin/env python3
"""Desktop control panel for the ESP32 LD19 LiDAR toolchain.

A thin Tkinter GUI over the project's make targets: pull/clear recordings off
the device, browse local .ldim files, run the 2D/3D visualizers and the
floorplan builder, and compile/flash the firmware. Every action shells out to
`make` so the GUI stays in lockstep with the command-line workflow (and reuses
its venv handling, LDIM/HOST overrides, and mcap caching).

Run it with python3.13 (the system python3 ships without tkinter):
    make gui      # or: python3.13 lidar_gui.py
"""
import json
import os
import queue
import signal
import subprocess
import threading
import urllib.error
import urllib.request
from pathlib import Path

import tkinter as tk
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parent
RECORDINGS = ROOT / "recordings"
DEFAULT_HOST = "lidar.local"


# ── Pure helpers (no Tk; unit-testable) ───────────────────────────────────────
def human_size(n):
    n = float(n)
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


def list_recordings():
    if not RECORDINGS.is_dir():
        return []
    return sorted(RECORDINGS.glob("*.ldim"), key=lambda p: p.name)


def build_make_cmd(target, ldim=None, host=None):
    cmd = ["make", target]
    if ldim:
        cmd.append(f"LDIM={ldim}")
    if host:
        cmd.append(f"HOST={host}")
    return cmd


def fetch_device_status(host, timeout=5):
    with urllib.request.urlopen(f"http://{host}/rec", timeout=timeout) as r:
        return json.load(r)


# Recording actions: (button label, make target, tooltip-ish note).
RECORDING_ACTIONS = [
    ("Info",          "dump",         "Duration + LD19/IMU sample counts"),
    ("View 2D",       "view",         "Static matplotlib scatter"),
    ("Floorplan PNG", "floorplan",    "Gyro deskew + ICP + loop closure"),
    ("Build MCAP",    "mcap",         "LaserScan + PointCloud2 + Imu"),
    ("Viz 2D",        "viz",          "Interactive PRBonn visualizer"),
    ("Viz 3D",        "viz3d",        "Tilt-compensated frame sequence"),
    ("Viz 3D static", "viz3d-static", "Merged single frame (tripod/pivot)"),
    ("Viz 3D merge",  "viz3d-merge",  "6-DoF drift-corrected merge (handheld)"),
]


class App:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.running = False
        self._on_done = None
        self.queue = queue.Queue()

        root.title("ESP32 LiDAR Control Panel")
        root.minsize(940, 680)

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.dev_status = tk.StringVar(value="device status unknown - press Check")
        self.sel_var = tk.StringVar(value="no recording selected")
        self.status_var = tk.StringVar(value="ready")

        self.action_buttons = []   # make-invoking buttons, toggled while running

        self._build_header()
        self._build_body()
        self._build_log()

        self.refresh_recordings()
        self.root.after(80, self.poll)

    # ── UI construction ──────────────────────────────────────────────────────
    def _build_header(self):
        bar = tk.Frame(self.root, bg="#222831")
        bar.pack(fill="x")
        tk.Label(bar, text="  ESP32 LiDAR Control Panel", bg="#222831",
                 fg="#eeeeee", font=("Helvetica", 16, "bold")).pack(
                     side="left", padx=10, pady=10)
        tk.Label(bar, text="LD19 + IMU toolchain", bg="#222831",
                 fg="#9aa0a6", font=("Helvetica", 11)).pack(side="left", pady=10)

    def _build_body(self):
        body = ttk.Frame(self.root, padding=10)
        body.pack(fill="both", expand=False)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        self._build_device(body)
        self._build_recordings(body)
        self._build_recording_actions(body)
        self._build_maintenance(body)

    def _mkbtn(self, parent, text, command, action=True, **grid):
        btn = ttk.Button(parent, text=text, command=command)
        btn.grid(sticky="ew", padx=4, pady=3, **grid)
        if action:
            self.action_buttons.append(btn)
        return btn

    def _build_device(self, body):
        f = ttk.LabelFrame(body, text="Device", padding=8)
        f.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="Host").grid(row=0, column=0, sticky="w", padx=4)
        ttk.Entry(f, textvariable=self.host_var).grid(
            row=0, column=1, columnspan=2, sticky="ew", padx=4, pady=3)

        ttk.Button(f, text="Check", command=self.check_device).grid(
            row=1, column=0, sticky="ew", padx=4, pady=3)
        self._mkbtn(f, "Pull recordings", self.pull, row=1, column=1)
        self._mkbtn(f, "Clear device", self.clear_device, row=1, column=2)

        ttk.Label(f, textvariable=self.dev_status, foreground="#555").grid(
            row=2, column=0, columnspan=3, sticky="w", padx=4, pady=(4, 0))

    def _build_recordings(self, body):
        f = ttk.LabelFrame(body, text="Local recordings", padding=8)
        f.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))
        f.columnconfigure(0, weight=1)
        f.rowconfigure(0, weight=1)

        tree = ttk.Treeview(f, columns=("size",), height=6, selectmode="browse")
        tree.heading("#0", text="file")
        tree.heading("size", text="size")
        tree.column("#0", width=200, anchor="w")
        tree.column("size", width=90, anchor="e", stretch=False)
        tree.grid(row=0, column=0, columnspan=3, sticky="nsew", padx=4, pady=3)
        tree.bind("<<TreeviewSelect>>", lambda _e: self.update_sel_label())
        self.tree = tree

        sb = ttk.Scrollbar(f, orient="vertical", command=tree.yview)
        sb.grid(row=0, column=3, sticky="ns")
        tree.configure(yscrollcommand=sb.set)

        ttk.Button(f, text="Refresh", command=self.refresh_recordings).grid(
            row=1, column=0, sticky="ew", padx=4, pady=3)
        ttk.Button(f, text="Open folder", command=self.open_recordings).grid(
            row=1, column=1, sticky="ew", padx=4, pady=3)
        ttk.Label(f, textvariable=self.sel_var, foreground="#555").grid(
            row=2, column=0, columnspan=4, sticky="w", padx=4, pady=(4, 0))

    def _build_recording_actions(self, body):
        f = ttk.LabelFrame(body, text="Selected recording", padding=8)
        f.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        for i, (label, target, note) in enumerate(RECORDING_ACTIONS):
            self._mkbtn(f, label, lambda t=target: self.run_on_selected(t),
                        row=i // 2, column=i % 2)

    def _build_maintenance(self, body):
        f = ttk.LabelFrame(body, text="Local + firmware", padding=8)
        f.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        self._mkbtn(f, "Clear local recordings", self.clear_local,
                    row=0, column=0, columnspan=2)
        self._mkbtn(f, "Compile firmware", self.compile_fw, row=1, column=0)
        self._mkbtn(f, "Flash firmware", self.flash_fw, row=1, column=1)
        ttk.Label(f, text="Flash needs the board on USB; viz/floorplan build "
                          "the venv on first run.",
                  foreground="#555", wraplength=360, justify="left").grid(
                      row=2, column=0, columnspan=2, sticky="w", padx=4, pady=(6, 0))

    def _build_log(self):
        f = ttk.LabelFrame(self.root, text="Output", padding=6)
        f.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        f.rowconfigure(0, weight=1)
        f.columnconfigure(0, weight=1)

        txt = tk.Text(f, height=12, wrap="none", state="disabled",
                      bg="#1e1e1e", fg="#e0e0e0", insertbackground="#e0e0e0",
                      font=("Menlo", 11), relief="flat")
        txt.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(f, orient="vertical", command=txt.yview)
        sb.grid(row=0, column=1, sticky="ns")
        txt.configure(yscrollcommand=sb.set)
        txt.tag_configure("cmd", foreground="#56b6c2")
        txt.tag_configure("ok", foreground="#98c379")
        txt.tag_configure("err", foreground="#e06c75")
        self.txt = txt

        ctl = ttk.Frame(f)
        ctl.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ctl.columnconfigure(0, weight=1)
        ttk.Label(ctl, textvariable=self.status_var, foreground="#555").grid(
            row=0, column=0, sticky="w", padx=4)
        self.stop_btn = ttk.Button(ctl, text="Stop", command=self.stop,
                                   state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=4)
        ttk.Button(ctl, text="Clear log", command=self.clear_log).grid(
            row=0, column=2, padx=4)

    # ── Command runner ───────────────────────────────────────────────────────
    def run(self, cmd, on_done=None):
        if self.running:
            messagebox.showinfo("Busy", "A command is already running.")
            return
        self.running = True
        self._on_done = on_done
        self._set_actions("disabled")
        self.stop_btn.configure(state="normal")
        self.status_var.set("running: " + " ".join(cmd))
        threading.Thread(target=self._worker, args=(cmd,), daemon=True).start()

    def _worker(self, cmd):
        self.queue.put(("cmd", "$ " + " ".join(cmd) + "\n"))
        try:
            self.proc = subprocess.Popen(
                cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True)
        except OSError as e:
            self.queue.put(("err", f"failed to launch: {e}\n"))
            self.queue.put(("done", 127))
            return
        for line in self.proc.stdout:
            self.queue.put(("out", line))
        code = self.proc.wait()
        self.proc = None
        self.queue.put(("done", code))

    def stop(self):
        p = self.proc
        if not p:
            return
        self.status_var.set("stopping...")
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    def poll(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "done":
                    self.running = False
                    self._set_actions("normal")
                    self.stop_btn.configure(state="disabled")
                    self.log(f"[exit {payload}]\n\n",
                             "ok" if payload == 0 else "err")
                    self.status_var.set("ready")
                    cb, self._on_done = self._on_done, None
                    if cb:
                        cb(payload)
                elif kind == "devstatus":
                    self.dev_status.set(payload)
                elif kind in ("cmd", "err"):
                    self.log(payload, kind)
                else:
                    self.log(payload)
        except queue.Empty:
            pass
        self.root.after(80, self.poll)

    def _set_actions(self, state):
        for b in self.action_buttons:
            b.configure(state=state)

    def log(self, text, tag=None):
        self.txt.configure(state="normal")
        self.txt.insert("end", text, (tag,) if tag else ())
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def clear_log(self):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

    # ── Actions ──────────────────────────────────────────────────────────────
    def host(self):
        return self.host_var.get().strip() or DEFAULT_HOST

    def selected_ldim(self):
        sel = self.tree.selection()
        if not sel:
            return None
        return f"recordings/{self.tree.item(sel[0], 'text')}"

    def update_sel_label(self):
        ldim = self.selected_ldim()
        self.sel_var.set(f"selected: {ldim}" if ldim
                         else "no recording selected")

    def refresh_recordings(self):
        self.tree.delete(*self.tree.get_children())
        for p in list_recordings():
            self.tree.insert("", "end", text=p.name,
                             values=(human_size(p.stat().st_size),))
        kids = self.tree.get_children()
        if kids:
            self.tree.selection_set(kids[-1])   # newest by name
            self.tree.see(kids[-1])
        self.update_sel_label()

    def run_on_selected(self, target):
        ldim = self.selected_ldim()
        if not ldim:
            messagebox.showinfo("No recording", "Select a recording first.")
            return
        self.run(build_make_cmd(target, ldim=ldim))

    def pull(self):
        self.run(build_make_cmd("pull-recordings", host=self.host()),
                 on_done=lambda _c: self.refresh_recordings())

    def clear_device(self):
        if not messagebox.askyesno(
                "Clear device",
                "Delete ALL recordings on the device?\n"
                "(the file currently being recorded is skipped)"):
            return
        self.run(build_make_cmd("clear-device", host=self.host()),
                 on_done=lambda _c: self.check_device())

    def clear_local(self):
        if not messagebox.askyesno(
                "Clear local",
                "Delete all local recordings and their derived "
                ".mcap/.png files?"):
            return
        self.run(build_make_cmd("clear-recordings"),
                 on_done=lambda _c: self.refresh_recordings())

    def compile_fw(self):
        self.run(build_make_cmd("compile"))

    def flash_fw(self):
        if not messagebox.askyesno("Flash",
                                   "Flash firmware to the board over USB?"):
            return
        self.run(build_make_cmd("flash"))

    def check_device(self):
        host = self.host()
        self.dev_status.set(f"checking {host}...")

        def work():
            try:
                info = fetch_device_status(host)
                n = len(info.get("files", []))
                free = info.get("free", 0) / 1e6
                rec = "  [RECORDING]" if info.get("recording") else ""
                self.queue.put(
                    ("devstatus", f"{host}: {n} file(s), {free:.1f} MB free{rec}"))
            except (urllib.error.URLError, OSError, ValueError) as e:
                self.queue.put(("devstatus", f"{host}: unreachable ({e})"))

        threading.Thread(target=work, daemon=True).start()

    def open_recordings(self):
        RECORDINGS.mkdir(exist_ok=True)
        subprocess.Popen(["open", str(RECORDINGS)])


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
