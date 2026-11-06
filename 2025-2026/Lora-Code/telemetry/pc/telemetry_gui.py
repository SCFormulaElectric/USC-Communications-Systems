#!/usr/bin/env python3
"""Tkinter telemetry viewer + diagnostics console (same link, decoder and logs as telemetry_cli).

    py telemetry_gui.py                     # pick the port in the window
    py telemetry_gui.py --port COM8         # connect straight away
    py telemetry_gui.py --demo              # simulated car, no hardware

- Every FAST / THERM / DIAG field is a row (built from the YAML); `valid` and `status` expand into
  one row per bit. Values stay until a new one arrives.
- A row turns red when its value changes and returns to normal after 0.5 s. "Highlight every
  update" highlights on every packet instead. Rows not updated for 3 s turn grey.
- Events tab: every EVENT record, newest first. Diagnostics tab: the CLI's commands (help).
"""
import argparse
import os
import queue
import sys
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import telemetry_cli
from decoder import Schema
from logger import TelemetryLogger
from lora_link import LoraLink
from telemetry_cli import HELP, Cli

NEW_S = 0.5        # highlight time (2 Hz)
HEX_FIELDS = {"fw_version": 4, "kern_status": 4, "bms_fault_flags": 8, "reg_seen": 4, "seen": 2,
              "raw_seen": 2, "raw_481": 4, "raw_666": 4, "raw_321": 4}
STALE_S = 3.0      # grey after this long without an update
TABLES = ("FAST", "THERM", "DIAG")


def fmt(v):
    if v is None:
        return ""
    if isinstance(v, bytes):
        return v.hex(" ")
    if isinstance(v, float):
        return f"{v:.1f}" if abs(v) >= 100 else f"{v:.2f}"
    if isinstance(v, list):
        return " ".join(v)
    return str(v)


class RowModel:
    """Display rows for one packet type, and which of them changed. No Tk: unit-testable."""

    def __init__(self, schema, type_name):
        self.s = schema
        self.type_name = type_name
        self.rows = []                     # (row_id, parent_id, label, field, bit, unit)
        pl = schema.payloads[schema.types[type_name]]
        for f in pl["fields"]:
            if f["type"] == "struct" or f["name"].startswith("reserved"):
                continue
            rid = f"{type_name}.{f['name']}"
            self.rows.append((rid, "", f["name"], f, None, f.get("unit", "")))
            if f.get("bits"):
                for b in schema.bit_tables[f["bits"]]:
                    self.rows.append((f"{rid}.{b['name']}", rid, f"  {b['bit']:>2} {b['name']}", f, b, ""))
        self.shown = {}                    # row_id -> (value text, raw text)
        self.changed_at = {}
        self.updated_at = {}

    def cell(self, d, f, bit):
        raw = d["fields"][f["name"]]
        if bit is not None:
            on = raw >> bit["bit"] & 1
            if f["bits"] == "valid_bits":
                return ("OK" if on else "timeout"), str(on)
            return ("yes" if on else "no"), str(on)
        if f.get("bits"):
            return f"0x{raw:04X}", str(raw)
        if f.get("enum") == "status_codes":
            return self.s.status_name(raw), str(raw)
        if f["name"] in HEX_FIELDS:
            return f"0x{raw:0{HEX_FIELDS[f['name']]}X}", str(raw)
        val = d["values"].get(f["name"], raw) if f["name"] in d["values"] else raw
        return fmt(val), fmt(raw)

    def update(self, d, t, every=False):
        """Apply a decoded packet; returns [(row_id, value, raw, highlight)]."""
        out = []
        for rid, _parent, _label, f, bit, _unit in self.rows:
            val, raw = self.cell(d, f, bit)
            changed = self.shown.get(rid) != (val, raw)
            if changed or every:
                self.changed_at[rid] = t
            self.shown[rid] = (val, raw)
            self.updated_at[rid] = t
            out.append((rid, val, raw, changed or every))
        return out


class GuiCli(Cli):
    """The CLI's command parser and result messages, printing into the GUI."""

    def __init__(self, link, schema, out):
        self.out = out
        super().__init__(link, schema)
        self.vt = False

    def say(self, msg):
        self.out(msg)


class App:
    def __init__(self, root, schema, args):
        self.root, self.s, self.args = root, schema, args
        self.q = queue.Queue()
        self.link = None
        self.logger = None
        self.cli = None
        self.models = {t: RowModel(schema, t) for t in TABLES}
        self.trees = {}
        self.ev_rows = {}
        root.title("SCFE LoRa telemetry")
        root.geometry("1180x760")
        style = ttk.Style(root)
        # Tk 8.6.9's ttk ignores Treeview tag colours without this
        for opt in ("foreground", "background"):
            style.map("Treeview", **{opt: [e for e in style.map("Treeview", query_opt=opt)
                                           if e[:2] != ("!disabled", "!selected")]})
        self.build()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        telemetry_cli.input = self.ask        # 'read 0x8F' asks before sending
        root.after(50, self.pump)
        root.after(100, self.decay)
        root.after(500, self.tick)
        if args.demo or args.port:
            root.after(100, self.connect)

    # ------------------------------------------------------------------ layout
    def build(self):
        top = ttk.Frame(self.root, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port = tk.StringVar(value="DEMO" if self.args.demo else (self.args.port or ""))
        self.port_box = ttk.Combobox(top, textvariable=self.port, width=10, values=self.ports())
        self.port_box.pack(side="left", padx=2)
        ttk.Button(top, text="↻", width=3, command=lambda: self.port_box.configure(values=self.ports())).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        ttk.Label(top, text="delay ms").pack(side="left")
        self.delay = tk.IntVar(value=self.args.delay_ms)
        ttk.Spinbox(top, from_=50, to=190, width=5, textvariable=self.delay).pack(side="left", padx=2)
        self.every = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Highlight every update", variable=self.every).pack(side="left", padx=8)
        self.status = ttk.Label(top, text="not connected", font=("Segoe UI", 10, "bold"))
        self.status.pack(side="left", padx=10)
        self.logpath = ttk.Label(self.root, text="", foreground="#666", padding=(6, 0))
        self.logpath.pack(fill="x")

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=4, pady=4)
        tel = ttk.Frame(nb)
        nb.add(tel, text="Telemetry")
        pw = ttk.PanedWindow(tel, orient="horizontal")
        pw.pack(fill="both", expand=True)
        left = ttk.Frame(pw)
        right = ttk.PanedWindow(pw, orient="vertical")
        pw.add(left, weight=1)
        pw.add(right, weight=1)
        self.trees["FAST"] = self.make_table(left, "FAST (10 Hz)")
        for t, title in (("THERM", "THERM (1 Hz)"), ("DIAG", "DIAG (1 Hz)")):
            fr = ttk.Frame(right)
            right.add(fr, weight=1)
            self.trees[t] = self.make_table(fr, title)
        for t in TABLES:
            tree = self.trees[t]
            for rid, parent, label, _f, _b, unit in self.models[t].rows:
                tree.insert(parent, "end", iid=rid, text=label, values=("", "", unit, ""), open=True)

        evf = ttk.Frame(nb)
        nb.add(evf, text="Events")
        self.ev = ttk.Treeview(evf, columns=("pc", "car", "name", "value"), show="headings")
        for c, w, txt in (("pc", 90, "PC time"), ("car", 110, "car uptime ms"), ("name", 260, "event"),
                          ("value", 120, "value")):
            self.ev.heading(c, text=txt)
            self.ev.column(c, width=w, anchor="w")
        self.ev.tag_configure("new", background="#ff9e9e")
        sb = ttk.Scrollbar(evf, command=self.ev.yview)
        self.ev.configure(yscrollcommand=sb.set)
        self.ev.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        dg = ttk.Frame(nb, padding=4)
        nb.add(dg, text="Diagnostics")
        row = ttk.Frame(dg)
        row.pack(fill="x")
        self.cmd = ttk.Entry(row, font=("Consolas", 10))
        self.cmd.pack(side="left", fill="x", expand=True)
        self.cmd.bind("<Return>", lambda e: self.send_cmd())
        ttk.Button(row, text="Send", command=self.send_cmd).pack(side="left", padx=4)
        ttk.Button(row, text="Help", command=lambda: self.log(HELP)).pack(side="left")
        self.out = scrolledtext.ScrolledText(dg, height=20, font=("Consolas", 10), state="disabled")
        self.out.pack(fill="both", expand=True, pady=4)
        self.log("Diagnostics need the Diag build on the car. Type a command (e.g. ping, read N_MAX) "
                 "and press Enter; 'help' lists them.")

    def make_table(self, parent, title):
        ttk.Label(parent, text=title, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        tree = ttk.Treeview(parent, columns=("value", "raw", "unit", "age"), show="tree headings")
        tree.heading("#0", text="field")
        tree.column("#0", width=190)
        for c, w in (("value", 150), ("raw", 110), ("unit", 50), ("age", 60)):
            tree.heading(c, text=c if c != "age" else "age s")
            tree.column(c, width=w, anchor="e" if c != "unit" else "w")
        tree.tag_configure("new", background="#ff9e9e")
        tree.tag_configure("stale", foreground="#999999")
        sb = ttk.Scrollbar(parent, command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        return tree

    # ------------------------------------------------------------------ connection
    @staticmethod
    def ports():
        try:
            from serial.tools import list_ports
            return [p.device for p in list_ports.comports()] + ["DEMO"]
        except Exception:
            return ["DEMO"]

    def toggle_connect(self):
        self.disconnect() if self.link else self.connect()

    def connect(self):
        port = self.port.get().strip()
        if not port:
            messagebox.showinfo("Port", "Pick the PC LoRa module's COM port (or DEMO).")
            return
        try:
            self.logger = None if self.args.no_log else TelemetryLogger(self.args.log_dir, self.s)
            factory = None
            if port.upper() == "DEMO":
                from sim_port import SimPort
                factory = lambda *a, **k: SimPort(self.s)
            self.link = LoraLink(port, self.s, delay_ms=int(self.delay.get()), logger=self.logger,
                                 serial_factory=factory)
        except Exception as e:
            self.link = None
            messagebox.showerror("Connect", f"Could not open {port}:\n{e}")
            return
        self.link.listeners.append(self.q.put)
        self.cli = GuiCli(self.link, self.s, lambda m: self.q.put({"kind": "log", "text": m}))
        self.link.start()
        self.conn_btn.configure(text="Disconnect")
        if self.logger:
            self.logpath.configure(text=f"logging to {self.logger.raw_path} and {self.logger.dir}")

    def disconnect(self):
        link, self.link = self.link, None
        if link:
            if link.sender.session:
                link.submit("STOP_ALL")
                t_end = time.time() + 2
                while time.time() < t_end and link.sender.session:
                    self.root.update()
                    time.sleep(0.05)
            link.stop()
        if self.logger:
            self.logger.close()
            self.logger = None
        self.conn_btn.configure(text="Connect")
        self.status.configure(text="not connected", foreground="black")

    def on_close(self):
        self.disconnect()
        self.root.destroy()

    # ------------------------------------------------------------------ updates (Tk thread only)
    def pump(self):
        try:
            for _ in range(500):
                ev = self.q.get_nowait()
                k = ev["kind"]
                if k == "packet":
                    self.on_packet(ev)
                elif k == "log":
                    self.log(ev["text"])
        except queue.Empty:
            pass
        self.root.after(50, self.pump)

    def on_packet(self, ev):
        d, t = ev["packet"], ev["t"]
        name = d["type_name"]
        if name in self.models:
            tree = self.trees[name]
            for rid, val, raw, hl in self.models[name].update(d, t, self.every.get()):
                old = tree.item(rid, "values")
                tree.item(rid, values=(val, raw, old[2] if len(old) > 2 else "", "0.0"),
                          tags=("new",) if hl else ())
        elif name == "EVENT":
            f = d["fields"]
            for r in f["records"][:f["count"]]:
                iid = self.ev.insert("", 0, values=(time.strftime("%H:%M:%S"), d["uptime_ms"] - r["age_ms"],
                                                    self.s.event_name(r["code"]), r["value"]), tags=("new",))
                self.ev_rows[iid] = t
            for old in self.ev.get_children("")[2000:]:
                self.ev.delete(old)

    def decay(self):
        now = time.perf_counter()
        for t, m in self.models.items():
            tree = self.trees[t]
            for rid, ct in list(m.changed_at.items()):
                if now - ct >= NEW_S:
                    tags = ("stale",) if now - m.updated_at.get(rid, now) > STALE_S else ()
                    tree.item(rid, tags=tags)
                    del m.changed_at[rid]
        for iid, ct in list(self.ev_rows.items()):
            if now - ct >= NEW_S:
                if self.ev.exists(iid):
                    self.ev.item(iid, tags=())
                del self.ev_rows[iid]
        self.root.after(100, self.decay)

    def tick(self):
        now = time.perf_counter()
        for t, m in self.models.items():
            tree = self.trees[t]
            for rid, ut in m.updated_at.items():
                age = now - ut
                tree.set(rid, "age", f"{age:.1f}")
                if rid not in m.changed_at:
                    tree.item(rid, tags=("stale",) if age > STALE_S else ())
        if self.link:
            st = self.link.stats()
            ok = st["state"] == "OK"
            self.status.configure(
                text=f"{st['state']}   {st['pps']:.1f} pkt/s   loss {st['loss_pct']:.1f}%   "
                     f"rx {st['received']}  missing {st['missing']}  bad CRC {st['bad_crc']}"
                     + ("   session" if st["session"] else ""),
                foreground="#1a7f37" if ok else "#b42318")
        self.root.after(500, self.tick)

    # ------------------------------------------------------------------ diagnostics
    def log(self, text):
        self.out.configure(state="normal")
        self.out.insert("end", time.strftime("%H:%M:%S  ") + text + "\n")
        self.out.see("end")
        self.out.configure(state="disabled")

    def send_cmd(self):
        line = self.cmd.get().strip()
        if not line:
            return
        self.cmd.delete(0, "end")
        self.log("> " + line)
        if not self.cli:
            self.log("not connected")
            return
        if line.split()[0].lower() in ("quit", "exit"):
            self.on_close()
            return
        try:
            self.cli.handle(line)
        except (IndexError, ValueError, KeyError) as e:
            self.log(f"? {e} (type help)")

    def ask(self, prompt=""):
        return "y" if messagebox.askyesno("Are you sure?", prompt) else "n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--demo", action="store_true", help="simulated car, no hardware")
    ap.add_argument("--delay-ms", type=int, default=110)
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--no-log", action="store_true")
    a = ap.parse_args()
    root = tk.Tk()
    App(root, Schema(), a)
    root.mainloop()


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
