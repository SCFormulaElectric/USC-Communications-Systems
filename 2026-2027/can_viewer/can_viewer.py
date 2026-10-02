#!/usr/bin/env python3
"""
CAN Viewer + DBC Decoder (Tkinter, cross-platform)

Shows every CAN frame on the bus, looks its ID up in one or more .dbc files,
and decodes it into named signals with real units, enum names, and comments.

Layout
  Live tab   : Node -> Message -> Signal tree. One row per message ID, updated
               in place (like PCAN-View "fixed" mode). Every message gets its
               own colour, rows flash blue when a value changes, red when a
               value is outside the DBC range, grey when the message goes stale.
  Trace tab  : scrolling log of frames in arrival order, same colours.
  Detail pane: click any message or signal to see bit layout, scale/offset,
               range, enum table, DBC comment, and decoded fault bits.

Sources
  serial text: STM32 (or any MCU) printing "ID: 0x0A3 STD DLC: 8 Data: .." lines
               over its USB COM port; channel "COM8" or "COM8@921600"
  demo       : simulated traffic generated from the loaded DBC (no hardware)
  pcan / kvaser / slcan / socketcan / gs_usb / vector / ixxat : real adapters
               via python-can
  Replay log : .asc .blf .csv .log (candump) .trc via python-can LogReader

Requires:  pip install cantools python-can pyserial
           (+ the vendor driver for your adapter, e.g. PCAN-Basic for PCAN)
"""

import colorsys
import json
import math
import os
import queue
import random
import re
import sys
import threading
import time
from collections import deque

import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog, messagebox

try:
    import cantools
except ImportError:
    sys.exit("Missing dependency: pip install cantools python-can")
try:
    import can
except ImportError:
    can = None  # demo mode still works without python-can
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # only needed for the "serial text" source


# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
GUI_TICK_MS = 50            # how often the GUI drains the RX queue
TREE_REFRESH_S = 0.10       # how often live rows are redrawn (10 Hz is plenty)
TRACE_MAX_ROWS = 1500       # rows kept in the trace view
TRACE_ROWS_PER_TICK = 150   # max rows inserted per tick (keeps UI responsive)
CHANGE_FLASH_S = 0.6        # how long a changed value stays highlighted
STALE_DEFAULT_S = 1.0       # stale threshold when the DBC has no cycle time
STALE_FACTOR = 5            # stale if age > STALE_FACTOR * expected period
FAULT_BITS_FILE = "fault_bits.json"

IS_WIN = sys.platform.startswith("win")
SERIAL_TEXT = "serial text (STM32)"
SERIAL_BAUD = 115200        # UART baud of the board's printf output; override with COM8@921600
INTERFACES = {
    SERIAL_TEXT: "COM8" if IS_WIN else "/dev/ttyACM0",
    "demo (simulated)": "",
    "pcan": "PCAN_USBBUS1",
    "kvaser": "0",
    "slcan": "COM3" if IS_WIN else "/dev/ttyACM0",
    "socketcan": "can0",
    "gs_usb": "0",
    "vector": "0",
    "ixxat": "0",
}
BITRATES = ["125000", "250000", "500000", "1000000"]
NODE_DESC = {"INV": "Inverter", "VCU": "Vehicle Control Unit",
             "BMS": "Battery Management System"}
UNKNOWN_NODE = "Not in DBC"


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------
def fmt_id(fid, ext):
    return f"0x{fid:08X}" if ext else f"0x{fid:03X}"


def short_unit(unit):
    """Cascadia DBCs write units as 'current:A' -> show just 'A'."""
    unit = unit or ""
    return unit.split(":", 1)[1] if ":" in unit else unit


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data)


def decimals_for(scale):
    """0.1 -> 1 decimal, 0.003 -> 3 decimals, 1 -> 0 decimals."""
    try:
        s = abs(float(scale))
    except (TypeError, ValueError):
        return 2
    if s == 0 or (s >= 1 and s.is_integer()):
        return 0
    return min(6, max(0, -math.floor(math.log10(s))))


def message_colours(fid):
    """Stable, well-spread pastel colour per frame ID (golden-ratio hue walk)."""
    hue = (fid * 0.618033988749895) % 1.0
    strong = colorsys.hls_to_rgb(hue, 0.80, 0.60)
    light = colorsys.hls_to_rgb(hue, 0.93, 0.50)
    to_hex = lambda rgb: "#%02x%02x%02x" % tuple(int(c * 255) for c in rgb)
    return to_hex(strong), to_hex(light)


def is_named(v):
    """cantools returns NamedSignalValue for enum signals."""
    return hasattr(v, "name") and hasattr(v, "value")


def is_fault_word(sig):
    return "fault" in sig.name.lower() and sig.length >= 8 and not sig.choices


def frame_bits(dlc, ext):
    """Approximate on-wire bits incl. ~20% stuffing, for bus-load estimate."""
    return int(((67 if ext else 47) + 8 * dlc) * 1.2)


def signal_bit_positions(sig):
    """Return the set of absolute bit indices (byte*8 + bit) a signal occupies."""
    bits = set()
    if sig.byte_order == "little_endian":
        for i in range(sig.length):
            bits.add(sig.start + i)
    else:  # Motorola: DBC start bit is the MSB, walk the 'sawtooth'
        b = sig.start
        for _ in range(sig.length):
            bits.add(b)
            b = b + 15 if b % 8 == 0 else b - 1
    return bits


# ----------------------------------------------------------------------------
# DBC store: merges any number of DBC files into one ID -> message lookup
# ----------------------------------------------------------------------------
class DbcStore:
    def __init__(self):
        self.files = []
        self.by_key = {}            # (frame_id, is_extended) -> (Message, path)
        self.fault_bits = {}

    def load(self, path):
        db = cantools.database.load_file(path, strict=False)
        warnings = []
        for m in db.messages:
            if m.name.startswith("VECTOR__INDEPENDENT"):
                continue
            key = (m.frame_id, bool(m.is_extended_frame))
            if key in self.by_key and self.by_key[key][1] != path:
                old = self.by_key[key]
                warnings.append(f"{fmt_id(*key)}: {old[0].name} "
                                f"({os.path.basename(old[1])}) replaced by "
                                f"{m.name} ({os.path.basename(path)})")
            self.by_key[key] = (m, path)
        if path not in self.files:
            self.files.append(path)
        return warnings

    def clear(self):
        self.files.clear()
        self.by_key.clear()

    def lookup(self, fid, ext):
        hit = self.by_key.get((fid, bool(ext)))
        return hit[0] if hit else None

    def load_fault_bits(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            self.fault_bits = {k: v for k, v in raw.items()
                               if not k.startswith("_") and isinstance(v, dict)}
        except FileNotFoundError:
            self.fault_bits = {}

    def fault_names(self, sig_name, word):
        """Decode a fault word into ['bit 3: name', ...]."""
        table = (self.fault_bits.get(sig_name)
                 or self.fault_bits.get(sig_name.replace("Diag_Run_Faults", "Run_Fault"))
                 or {})
        out = []
        for bit in range(32):
            if word >> bit & 1:
                out.append(f"bit {bit}: {table.get(str(bit), '(see manual)')}")
        return out


# ----------------------------------------------------------------------------
# Per-message live state
# ----------------------------------------------------------------------------
class MsgState:
    def __init__(self, key, dbmsg):
        self.key = key
        self.fid, self.ext = key
        self.dbmsg = dbmsg
        self.node = (dbmsg.senders[0] if dbmsg and dbmsg.senders else
                     ("Unassigned" if dbmsg else UNKNOWN_NODE))
        self.count = 0
        self.last_rx = 0.0
        self.last_ts = None
        self.period = None          # EMA of measured period, seconds
        self.data = b""
        self.dlc = 0
        self.values = {}            # signal -> scaled / named value (accumulates mux pages)
        self.raws = {}              # signal -> raw integer
        self.shown = {}             # signal -> last displayed string
        self.changed_at = {}        # signal -> time of last change
        self.error = None
        self.dirty = False

    @property
    def expected_s(self):
        ct = getattr(self.dbmsg, "cycle_time", None) if self.dbmsg else None
        return ct / 1000.0 if ct else None


# ----------------------------------------------------------------------------
# Frame sources (each runs in its own thread and pushes into a Queue)
# ----------------------------------------------------------------------------
class SourceThread(threading.Thread):
    def __init__(self, q):
        super().__init__(daemon=True)
        self.q = q
        self.stop_evt = threading.Event()

    def emit(self, kind, payload=None):
        """Every queue item carries its source, so the GUI can drop items from
        a source that has already been disconnected/replaced."""
        self.q.put((kind, payload, time.time(), self))

    def stop(self, wait=1.0):
        """Signal the thread and wait for it, so hardware is really released
        before a new connection is opened on the same channel."""
        self.stop_evt.set()
        if wait and self.is_alive() and threading.current_thread() is not self:
            self.join(wait)


class BusReader(SourceThread):
    def __init__(self, q, bus):
        super().__init__(q)
        self.bus = bus

    def run(self):
        try:
            while not self.stop_evt.is_set():
                m = self.bus.recv(0.1)
                if m is None:
                    continue
                if m.is_error_frame:
                    self.emit("errframe")
                else:
                    self.emit("rx", m)
        except Exception as e:  # adapter unplugged, driver error, ...
            self.emit("error", f"Bus error: {e}")
        finally:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.emit("stopped")


class ReplayReader(SourceThread):
    def __init__(self, q, path):
        super().__init__(q)
        self.path = path

    def run(self):
        reader = None
        try:
            reader = can.LogReader(self.path)
            for m in can.MessageSync(reader, timestamps=True, gap=0.0001):
                if self.stop_evt.is_set():
                    break
                if not m.is_error_frame:
                    self.emit("rx", m)
            else:
                self.emit("info", "Replay finished")
        except Exception as e:
            self.emit("error", f"Replay error: {e}")
        finally:
            if reader is not None and hasattr(reader, "stop"):
                try:
                    reader.stop()       # close the log file
                except Exception:
                    pass
            self.emit("stopped")


class SerialTextReader(SourceThread):
    """Reads frames that a microcontroller (e.g. STM32 Nucleo + CAN transceiver)
    prints over its USB virtual COM port, one per line:
        ID: 0x0A3 STD DLC: 8 Data: F3 C5 35 1F 10 00 00 00
        ID: 0x00000800 EXT DLC: 8 Data: 00 00 00 00 00 00 00 00
    Lines that don't match (boot banners, debug prints) are ignored."""

    LINE_RE = re.compile(r"ID:\s*(?:0x)?([0-9A-Fa-f]+)\s+(STD|EXT)\s+DLC:\s*(\d+)"
                         r"\s+Data:\s*((?:[0-9A-Fa-f]{2}\s*)*)", re.IGNORECASE)

    def __init__(self, q, port, baud):
        super().__init__(q)
        if not port:
            raise ValueError("No serial port given (e.g. COM8)")
        self.port, self.baud = port, baud
        self.ser = serial.Serial(port, baud, timeout=0.1)   # raises now if port is busy/missing

    @classmethod
    def parse(cls, line):
        m = cls.LINE_RE.search(line)
        if not m:
            return None
        data = bytes.fromhex(m.group(4))
        dlc = min(int(m.group(3)), 8)
        ext = m.group(2).upper() == "EXT"
        if can is not None:
            return can.Message(timestamp=time.time(), arbitration_id=int(m.group(1), 16),
                               is_extended_id=ext, dlc=dlc, data=data[:dlc])
        return FakeMsg(int(m.group(1), 16), ext, data[:dlc])

    def run(self):
        buf = b""
        try:
            self.ser.reset_input_buffer()     # drop half a line from before we connected
            while not self.stop_evt.is_set():
                buf += self.ser.read(self.ser.in_waiting or 1)
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    msg = self.parse(raw.decode("ascii", "replace"))
                    if msg is not None:
                        self.emit("rx", msg)
                if len(buf) > 4096:           # wrong baud rate -> garbage with no newlines
                    buf = b""
        except Exception as e:                # board unplugged, port grabbed by another app
            self.emit("error", f"Serial error on {self.port}: {e}")
        finally:
            try:
                self.ser.close()
            except Exception:
                pass
            self.emit("stopped")


def find_stm32_port():
    """COM port of an ST-LINK virtual COM port (USB VID 0x0483), if one is plugged in."""
    if serial is None:
        return None
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483:
            return p.device
    return None


class FakeMsg:
    """Same attributes the GUI uses from can.Message, so demo works w/o python-can."""
    __slots__ = ("arbitration_id", "is_extended_id", "dlc", "data", "timestamp",
                 "is_error_frame")

    def __init__(self, fid, ext, data):
        self.arbitration_id, self.is_extended_id = fid, ext
        self.data, self.dlc = data, len(data)
        self.timestamp, self.is_error_frame = time.time(), False


class DemoSource(SourceThread):
    """Generates plausible traffic for every periodic message in the DBC."""

    def __init__(self, q, messages):
        super().__init__(q)
        self.msgs = [m for m in messages if m.cycle_time]
        if not self.msgs:  # DBC without cycle times: send everything at 100 ms
            self.msgs = list(messages)
        self.state = {}
        self.mux_idx = {}

    def _initial(self, s):
        if is_fault_word(s):
            return 0
        if s.choices:
            return random.choice(list(s.choices.keys()))
        lo, hi = self._range(s)
        return (lo + hi) / 2 if s.length > 1 else 0

    def _range(self, s):
        if s.minimum is not None and s.maximum is not None and s.maximum > s.minimum:
            return float(s.minimum), float(s.maximum)
        span = (1 << s.length) - 1
        raw_lo, raw_hi = (-(span + 1) // 2, span // 2) if s.is_signed else (0, span)
        return raw_lo * s.scale + s.offset, raw_hi * s.scale + s.offset

    def _step(self, s, v):
        if is_fault_word(s):   # mostly healthy, with the odd fault blip
            if v:
                return 0 if random.random() < 0.02 else v
            return (1 << random.randrange(s.length)) if random.random() < 0.0005 else 0
        if random.random() < 0.5:   # not every value moves every frame
            return v
        if s.choices:
            return random.choice(list(s.choices.keys())) if random.random() < 0.01 else v
        if s.length == 1:
            return 1 - v if random.random() < 0.005 else v
        lo, hi = self._range(s)
        v += random.uniform(-1, 1) * (hi - lo) * 0.002
        return min(hi, max(lo, v))

    def _payload(self, m):
        st = self.state.setdefault(m.name, {s.name: self._initial(s) for s in m.signals})
        for s in m.signals:
            st[s.name] = self._step(s, st[s.name])
        sigs = m.signals
        if m.is_multiplexed():
            muxer = next(s for s in m.signals if s.is_multiplexer)
            ids = sorted({i for s in m.signals for i in (s.multiplexer_ids or [])})
            idx = self.mux_idx.get(m.name, 0) % max(1, len(ids))
            self.mux_idx[m.name] = idx + 1
            mux_val = ids[idx] if ids else 0
            st[muxer.name] = mux_val
            sigs = [s for s in m.signals
                    if s.is_multiplexer or s.multiplexer_ids is None
                    or mux_val in s.multiplexer_ids]
        try:
            return m.encode({s.name: st[s.name] for s in sigs}, strict=False)
        except Exception:
            return bytes(random.getrandbits(8) for _ in range(m.length))

    def run(self):
        now = time.time()
        due = {m.name: now + random.random() * 0.05 for m in self.msgs}
        while not self.stop_evt.is_set():
            now = time.time()
            for m in self.msgs:
                if now >= due[m.name]:
                    period = (m.cycle_time or 100) / 1000.0
                    due[m.name] = now + period * random.uniform(0.95, 1.05)
                    self.emit("rx", FakeMsg(m.frame_id, bool(m.is_extended_frame),
                                            self._payload(m)))
            time.sleep(0.002)
        self.emit("stopped")


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------
class CanViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN Viewer - DBC Decoder")
        self.root.geometry("1400x820")
        self.root.minsize(1000, 600)

        self.dbc = DbcStore()
        self.dbc.load_fault_bits(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              FAULT_BITS_FILE))
        self.q = queue.Queue()
        self.source = None
        self.logger = None
        self.states = {}             # key -> MsgState
        self.node_keys = {}          # node -> [keys]
        self.trace_pending = deque(maxlen=TRACE_MAX_ROWS)
        self.colour_tags = set()
        self.last_refresh = 0.0
        self.rate_window = deque()   # (time, bits) for msg/s + bus load
        self.err_frames = 0
        self.paused = False

        self._build_style()
        self._build_toolbar()
        self._build_body()
        self._build_statusbar()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(GUI_TICK_MS, self.tick)

    # ---------------------------------------------------------------- styling
    def _build_style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")     # same look on Windows / macOS / Linux
        base = tkfont.nametofont("TkDefaultFont")
        self.f_bold = base.copy()
        self.f_bold.configure(weight="bold")
        self.f_node = base.copy()
        self.f_node.configure(weight="bold", size=base.cget("size") + 1)
        self.f_mono = tkfont.nametofont("TkFixedFont").copy()
        rowh = max(22, self.f_node.metrics("linespace") + 6)
        style.configure("Treeview", rowheight=rowh)
        style.configure("Treeview.Heading", font=self.f_bold)
        style.configure("Status.TLabel", padding=(6, 2))

    # ---------------------------------------------------------------- toolbar
    def _build_toolbar(self):
        bar = ttk.Frame(self.root, padding=(6, 6, 6, 2))
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Button(bar, text="Load DBC...", command=self.load_dbc).pack(side=tk.LEFT)
        ttk.Button(bar, text="Clear DBCs", command=self.clear_dbcs).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(bar, text="Interface").pack(side=tk.LEFT)
        self.iface_var = tk.StringVar(value=SERIAL_TEXT)
        iface = ttk.Combobox(bar, textvariable=self.iface_var, values=list(INTERFACES),
                             width=18, state="readonly")
        iface.pack(side=tk.LEFT, padx=4)
        iface.bind("<<ComboboxSelected>>",
                   lambda e: self.chan_var.set(self._default_channel(self.iface_var.get())))

        ttk.Label(bar, text="Channel").pack(side=tk.LEFT)
        self.chan_var = tk.StringVar(value=self._default_channel(SERIAL_TEXT))
        ttk.Entry(bar, textvariable=self.chan_var, width=14).pack(side=tk.LEFT, padx=4)

        ttk.Label(bar, text="Bitrate").pack(side=tk.LEFT)
        self.rate_var = tk.StringVar(value="500000")
        ttk.Combobox(bar, textvariable=self.rate_var, values=BITRATES, width=9).pack(side=tk.LEFT, padx=4)

        bar = ttk.Frame(self.root, padding=(6, 2, 6, 2))      # second toolbar row
        bar.pack(side=tk.TOP, fill=tk.X)
        self.conn_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(bar, text="Replay log...", command=self.start_replay).pack(side=tk.LEFT)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        self.pause_btn = ttk.Button(bar, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side=tk.LEFT)
        ttk.Button(bar, text="Clear", command=self.clear_view).pack(side=tk.LEFT, padx=4)
        self.rec_btn = ttk.Button(bar, text="Record...", command=self.toggle_record)
        self.rec_btn.pack(side=tk.LEFT)

        ttk.Label(bar, text="Filter (ID / name / signal)").pack(side=tk.LEFT, padx=(14, 4))
        self.filter_var = tk.StringVar()
        ent = ttk.Entry(bar, textvariable=self.filter_var, width=24)
        ent.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.filter_var.trace_add("write", lambda *a: self.apply_filter())

    # ---------------------------------------------------------------- body
    def _build_body(self):
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.tabs = ttk.Notebook(paned)
        paned.add(self.tabs, weight=4)

        # --- Live tree
        live = ttk.Frame(self.tabs)
        self.tabs.add(live, text="  Live (by message)  ")
        btns = ttk.Frame(live)
        btns.pack(fill=tk.X, pady=(4, 2))
        ttk.Button(btns, text="Expand all", command=lambda: self.set_open(True)).pack(side=tk.LEFT)
        ttk.Button(btns, text="Collapse signals", command=lambda: self.set_open(False)).pack(side=tk.LEFT, padx=4)
        self.flash_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(btns, text="Highlight changes", variable=self.flash_var).pack(side=tk.LEFT, padx=8)
        self.legend = ttk.Label(btns, text="blue = changed   red = out of range / fault   "
                                           "grey = stale")
        self.legend.pack(side=tk.LEFT, padx=10)

        cols = ("id", "raw", "value", "unit", "count", "period", "status")
        self.tree = ttk.Treeview(live, columns=cols, selectmode="browse")
        heads = {"#0": ("Node / Message / Signal", 320), "id": ("ID", 110),
                 "raw": ("Data / Raw", 210), "value": ("Decoded value", 230),
                 "unit": ("Unit", 70), "count": ("Count", 70),
                 "period": ("Period (meas / DBC)", 140), "status": ("Status", 110)}
        for c, (txt, w) in heads.items():
            self.tree.heading(c, text=txt, anchor=tk.W)
            self.tree.column(c, width=w, anchor=tk.W, stretch=(c in ("#0", "value")))
        ysb = ttk.Scrollbar(live, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=ysb.set)
        ysb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<<TreeviewSelect>>", lambda e: self.show_detail())

        # status tags only set foreground; colour tags only set background,
        # so they never fight over the same option
        self.tree.tag_configure("node", font=self.f_node, background="#d9dde3")
        self.tree.tag_configure("msgfont", font=self.f_bold)
        self.tree.tag_configure("changed", foreground="#0b57d0")
        self.tree.tag_configure("oor", foreground="#c5221f")
        self.tree.tag_configure("stale", foreground="#8a8f98")
        self.tree.tag_configure("unknown", background="#eeeeee")

        # --- Trace
        trace = ttk.Frame(self.tabs)
        self.tabs.add(trace, text="  Trace (time order)  ")
        tbar = ttk.Frame(trace)
        tbar.pack(fill=tk.X, pady=(4, 2))
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(tbar, text="Auto-scroll", variable=self.autoscroll).pack(side=tk.LEFT)
        ttk.Label(tbar, text="   (at high bus rates the trace is sub-sampled; "
                             "the Live tab and recordings see every frame)").pack(side=tk.LEFT)
        tcols = ("time", "id", "name", "dlc", "data", "decoded")
        self.trace = ttk.Treeview(trace, columns=tcols, show="headings")
        for c, txt, w in (("time", "Time", 110), ("id", "ID", 95), ("name", "Message", 230),
                          ("dlc", "DLC", 45), ("data", "Data", 210), ("decoded", "Decoded", 600)):
            self.trace.heading(c, text=txt, anchor=tk.W)
            self.trace.column(c, width=w, anchor=tk.W, stretch=(c == "decoded"))
        tsb = ttk.Scrollbar(trace, orient=tk.VERTICAL, command=self.trace.yview)
        self.trace.configure(yscrollcommand=tsb.set)
        tsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.trace.pack(fill=tk.BOTH, expand=True)
        self.trace.tag_configure("unknown", background="#eeeeee")

        # --- Detail pane
        detail = ttk.LabelFrame(paned, text="Details", padding=4)
        paned.add(detail, weight=1)
        self.detail = tk.Text(detail, wrap=tk.WORD, width=42, font=self.f_mono,
                              relief=tk.FLAT, state=tk.DISABLED)
        self.detail.pack(fill=tk.BOTH, expand=True)
        self.detail.tag_configure("h", font=self.f_bold)
        self.detail.tag_configure("on", foreground="#0b57d0")

    def _build_statusbar(self):
        self.status_var = tk.StringVar(value="Load a DBC, pick an interface, press Connect "
                                             "(demo works without hardware).")
        ttk.Label(self.root, textvariable=self.status_var, style="Status.TLabel",
                  relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

    # ---------------------------------------------------------------- DBC
    def load_dbc(self):
        paths = filedialog.askopenfilenames(title="Select DBC file(s)",
                                            filetypes=[("DBC files", "*.dbc"), ("All", "*.*")])
        warnings = []
        for p in paths:
            try:
                warnings += self.dbc.load(p)
            except Exception as e:
                messagebox.showerror("DBC error", f"{os.path.basename(p)}:\n{e}")
        if warnings:
            messagebox.showwarning(
                "Duplicate IDs",
                "These IDs exist in more than one DBC (the last one loaded wins).\n"
                "Cascadia DBC variants share IDs - usually load only ONE of them.\n\n"
                + "\n".join(warnings[:25]) + ("\n..." if len(warnings) > 25 else ""))
        if paths:
            self.rebind_all()

    def clear_dbcs(self):
        self.dbc.clear()
        self.rebind_all()

    def rebind_all(self):
        """Re-resolve every seen ID against the (new) DBC set and rebuild the tree."""
        old = dict(self.states)
        self.clear_view()
        for key, st in old.items():
            new = self._new_state(key)
            new.count, new.last_rx, new.last_ts, new.period = st.count, st.last_rx, st.last_ts, st.period
            if st.data:
                self._decode_into(new, st.data, st.dlc)
        # the demo simulates the DBC it was started with -> restart it on the new set
        if isinstance(self.source, DemoSource):
            self.stop_source()
            if self.dbc.by_key:
                self._start_source(DemoSource(self.q, self._dbc_messages()), "demo",
                                   "Disconnect", fresh=False)
        self.update_status()

    # ---------------------------------------------------------------- sources
    def _dbc_messages(self):
        return [m for m, _ in self.dbc.by_key.values()]

    def _drain_queue(self):
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass

    def _start_source(self, src, label, btn_text, fresh=True):
        """Single entry point for every source, so each session starts clean."""
        self.stop_source()
        self._drain_queue()
        if fresh:
            self.clear_view()
        self.rate_window.clear()
        self.err_frames = 0
        self.source = src
        self.source_label = label
        src.start()
        self.conn_btn.configure(text=btn_text)
        self.update_status()

    def toggle_connect(self):
        if self.source:
            self.stop_source()
            return
        iface = self.iface_var.get()
        if iface.startswith("demo"):
            msgs = self._dbc_messages()
            if not msgs:
                messagebox.showinfo("Demo", "Load a DBC first - the demo simulates its messages.")
                return
            self._start_source(DemoSource(self.q, msgs), "demo", "Disconnect")
            return
        if iface == SERIAL_TEXT:
            self._connect_serial_text()
            return
        if can is None:
            messagebox.showerror("python-can missing", "pip install python-can")
            return
        chan = self.chan_var.get().strip()
        try:
            bitrate = int(self.rate_var.get())
        except ValueError:
            messagebox.showerror("Bitrate", f"Invalid bitrate: {self.rate_var.get()!r}")
            return
        try:
            bus = can.Bus(interface=iface, channel=chan, bitrate=bitrate)
        except Exception as e:
            messagebox.showerror("Connect failed",
                                 f"{iface} / {chan}:\n{e}\n\n"
                                 "Check the adapter is plugged in, the vendor driver is "
                                 "installed, and the channel name is right.")
            return
        self._start_source(BusReader(self.q, bus),
                           f"{iface} {chan} @ {bitrate // 1000} kbit/s", "Disconnect")

    @staticmethod
    def _default_channel(iface):
        if iface == SERIAL_TEXT:
            return find_stm32_port() or INTERFACES[SERIAL_TEXT]
        return INTERFACES[iface]

    def _connect_serial_text(self):
        if serial is None:
            messagebox.showerror("pyserial missing", "pip install pyserial")
            return
        port, _, baud = self.chan_var.get().strip().partition("@")   # "COM8" or "COM8@921600"
        try:
            baud = int(baud) if baud else SERIAL_BAUD
            src = SerialTextReader(self.q, port, baud)
        except Exception as e:
            ports = "\n".join(f"  {p.device}  {p.description}"
                              for p in serial.tools.list_ports.comports()) or "  (none)"
            messagebox.showerror("Connect failed",
                                 f"{port} @ {baud}:\n{e}\n\nSerial ports on this computer:\n{ports}\n\n"
                                 "Close any other program using the port (STM32CubeIDE "
                                 "serial console, PuTTY, Arduino monitor...).")
            return
        self._start_source(src, f"{port} serial @ {baud} baud", "Disconnect")

    def start_replay(self):
        if can is None:
            messagebox.showerror("python-can missing", "pip install python-can")
            return
        path = filedialog.askopenfilename(
            title="Replay CAN log",
            filetypes=[("CAN logs", "*.asc *.blf *.csv *.log *.trc"), ("All", "*.*")])
        if not path:
            return
        self._start_source(ReplayReader(self.q, path),
                           f"replay {os.path.basename(path)}", "Stop")

    def stop_source(self):
        src, self.source = self.source, None
        if src:
            src.stop()      # waits for the thread so the adapter is released
        self.conn_btn.configure(text="Connect")

    def toggle_record(self):
        if self.logger:
            self.logger.stop()
            self.logger = None
            self.rec_btn.configure(text="Record...")
            return
        if can is None:
            messagebox.showerror("python-can missing", "pip install python-can")
            return
        path = filedialog.asksaveasfilename(
            title="Record to", defaultextension=".asc",
            filetypes=[("Vector ASC", "*.asc"), ("Vector BLF", "*.blf"),
                       ("CSV", "*.csv"), ("candump", "*.log")])
        if path:
            self.logger = can.Logger(path)
            self.rec_btn.configure(text="Stop recording")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.configure(text="Resume" if self.paused else "Pause")

    # ---------------------------------------------------------------- state / tree
    def clear_view(self):
        # rows hidden by the filter are *detached*, not children of the root,
        # so delete every known row explicitly or they linger and collide later
        doomed = [self.msg_iid(k) for k in self.states]
        doomed += [f"n:{n}" for n in self.node_keys]
        doomed += list(self.tree.get_children())
        for iid in doomed:
            if self.tree.exists(iid):
                self.tree.delete(iid)
        self.trace.delete(*self.trace.get_children())
        self.states.clear()
        self.node_keys.clear()
        self.trace_pending.clear()
        self.err_frames = 0
        self._set_detail([])

    @staticmethod
    def msg_iid(key):
        return f"m:{key[0]}:{int(key[1])}"

    @staticmethod
    def sig_iid(key, name):
        return f"s:{key[0]}:{int(key[1])}:{name}"

    def _colour_tag(self, fid):
        tag_m, tag_s = f"cm{fid}", f"cs{fid}"
        if tag_m not in self.colour_tags:
            strong, light = message_colours(fid)
            self.tree.tag_configure(tag_m, background=strong)
            self.tree.tag_configure(tag_s, background=light)
            self.trace.tag_configure(tag_m, background=light)
            self.colour_tags.add(tag_m)
        return tag_m, tag_s

    def _node_order(self, node):
        order = ["INV", "VCU", "BMS"]
        if node == UNKNOWN_NODE:
            return (2, node)
        return (0, order.index(node)) if node in order else (1, node)

    def _new_state(self, key):
        dbmsg = self.dbc.lookup(*key)
        st = MsgState(key, dbmsg)
        self.states[key] = st
        node = st.node
        node_iid = f"n:{node}"
        if not self.tree.exists(node_iid):
            label = f"{node}  -  {NODE_DESC.get(node, '')}" if node in NODE_DESC else node
            self.tree.insert("", tk.END, iid=node_iid, text=label, open=True, tags=("node",))
            self.node_keys[node] = []
        self.node_keys[node].append(key)

        miid = self.msg_iid(key)
        if dbmsg:
            tag_m, tag_s = self._colour_tag(key[0])
            self.tree.insert(node_iid, tk.END, iid=miid, text=dbmsg.name,
                             values=(fmt_id(*key),), open=True, tags=(tag_m, "msgfont"))
            for s in dbmsg.signals:
                self.tree.insert(miid, tk.END, iid=self.sig_iid(key, s.name), text=s.name,
                                 values=("", "", "-", short_unit(s.unit)), tags=(tag_s,))
        else:
            self.tree.insert(node_iid, tk.END, iid=miid, text="(unknown ID)",
                             values=(fmt_id(*key),), tags=("unknown", "msgfont"))
        self._place_node(node)
        return st

    def _matches(self, key):
        q = self.filter_var.get().strip().lower()
        if not q:
            return True
        st = self.states[key]
        hay = [fmt_id(*key).lower(), str(key[0]), st.node.lower()]
        if st.dbmsg:
            hay.append(st.dbmsg.name.lower())
            hay += [s.name.lower() for s in st.dbmsg.signals]
        return all(any(tok in h for h in hay) for tok in q.split())

    def _place_node(self, node):
        """Sort a node's messages by ID and detach the ones hidden by the filter."""
        node_iid = f"n:{node}"
        visible = sorted((k for k in self.node_keys[node] if self._matches(k)),
                         key=lambda k: (k[1], k[0]))
        for k in self.node_keys[node]:
            if k not in visible:
                self.tree.detach(self.msg_iid(k))
        for i, k in enumerate(visible):
            self.tree.move(self.msg_iid(k), node_iid, i)
        # order node rows; hide empty ones
        nodes = sorted(self.node_keys, key=self._node_order)
        idx = 0
        for n in nodes:
            n_iid = f"n:{n}"
            if self.tree.get_children(n_iid):
                self.tree.move(n_iid, "", idx)
                idx += 1
            else:
                self.tree.detach(n_iid)

    def apply_filter(self):
        for node in list(self.node_keys):
            self._place_node(node)

    def set_open(self, state):
        for node in self.tree.get_children(""):
            self.tree.item(node, open=True)
            for m in self.tree.get_children(node):
                self.tree.item(m, open=state)

    # ---------------------------------------------------------------- RX path
    def _decode_into(self, st, data, dlc):
        st.data, st.dlc, st.dirty = data, dlc, True
        if not st.dbmsg:
            return
        try:
            st.values.update(st.dbmsg.decode(data, decode_choices=True, scaling=True,
                                             allow_truncated=True))
            st.raws.update(st.dbmsg.decode(data, decode_choices=False, scaling=False,
                                           allow_truncated=True))
            st.error = None
        except Exception as e:
            st.error = str(e)

    def on_rx(self, m, rx_t):
        key = (m.arbitration_id, bool(m.is_extended_id))
        st = self.states.get(key) or self._new_state(key)
        if st.last_ts is not None:
            dt = m.timestamp - st.last_ts
            if dt > 0:
                st.period = dt if st.period is None else 0.9 * st.period + 0.1 * dt
        st.last_ts, st.last_rx = m.timestamp, rx_t
        st.count += 1
        self._decode_into(st, bytes(m.data), m.dlc)
        self.rate_window.append((rx_t, frame_bits(m.dlc, key[1])))
        self.trace_pending.append((m.timestamp, key, bytes(m.data), m.dlc))
        if self.logger and isinstance(m, can.Message):
            self.logger.on_message_received(m)

    def tick(self):
        # always reschedule, even if something below raises - otherwise one bad
        # frame silently freezes the whole GUI until restart
        try:
            self._tick()
        except Exception as e:
            self.status_var.set(f"Internal error: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.root.after(GUI_TICK_MS, self.tick)

    def _tick(self):
        errors = []
        for _ in range(20000):
            try:
                kind, payload, t, src = self.q.get_nowait()
            except queue.Empty:
                break
            if src is not self.source:
                continue        # leftover from a source we already disconnected
            if kind == "rx":
                if not self.paused:
                    self.on_rx(payload, t)
                elif self.logger and can and isinstance(payload, can.Message):
                    self.logger.on_message_received(payload)   # keep recording while paused
            elif kind == "errframe":
                self.err_frames += 1
            elif kind == "error":
                errors.append(payload)
            elif kind == "info":
                self.status_var.set(payload)
            elif kind == "stopped":
                self.source = None
                self.conn_btn.configure(text="Connect")

        now = time.time()
        if now - self.last_refresh >= TREE_REFRESH_S:
            self.last_refresh = now
            self.refresh_live(now)
            self.update_status(now)
        self.render_trace()
        for msg in errors:          # modal dialogs last, after state is consistent
            messagebox.showerror("CAN", msg)

    # ---------------------------------------------------------------- rendering
    def format_signal(self, s, v):
        if is_named(v):
            return str(v.name)
        if is_fault_word(s):
            w = int(v)
            if w == 0:
                return "0x0000  (no faults)"
            n = bin(w).count("1")
            return f"0x{w:04X}  ({n} fault bit{'s' if n > 1 else ''} set)"
        if isinstance(v, float):
            return f"{v:.{decimals_for(s.scale)}f}"
        return str(v)

    def refresh_live(self, now):
        for key, st in self.states.items():
            miid = self.msg_iid(key)
            exp = st.expected_s
            stale_after = max(STALE_DEFAULT_S, STALE_FACTOR * exp) if exp else STALE_DEFAULT_S
            stale = (now - st.last_rx) > stale_after

            if not st.dirty and not stale and not st.changed_at:
                continue

            if st.error:
                status = "DECODE ERR"
            elif not st.dbmsg:
                status = "not in DBC"
            elif st.dlc != st.dbmsg.length:
                status = f"DLC {st.dlc}!={st.dbmsg.length}"
            elif stale:
                status = "STALE"
            elif exp and st.period and st.period > 1.5 * exp:
                status = "SLOW"
            else:
                status = "OK"

            per = f"{st.period * 1000:.1f} ms" if st.period else "-"
            if exp:
                per += f" / {exp * 1000:g}"
            tags = list(self.tree.item(miid, "tags"))
            tags = [t for t in tags if t not in ("stale",)] + (["stale"] if stale else [])
            self.tree.item(miid, values=(fmt_id(*key), hex_bytes(st.data), "", "",
                                         st.count, per, status), tags=tags)

            msg_fault = False
            if st.dbmsg:
                for s in st.dbmsg.signals:
                    if s.name not in st.values:
                        continue
                    v = st.values[s.name]
                    txt = self.format_signal(s, v)
                    if st.shown.get(s.name) not in (None, txt):
                        st.changed_at[s.name] = now
                    st.shown[s.name] = txt
                    flags = []
                    if self.flash_var.get() and now - st.changed_at.get(s.name, 0) < CHANGE_FLASH_S:
                        flags.append("changed")
                    else:
                        st.changed_at.pop(s.name, None)
                    num = v.value if is_named(v) else v
                    if (not is_named(v) and isinstance(num, (int, float))
                            and s.minimum is not None and s.maximum is not None
                            and s.maximum > s.minimum
                            and not (s.minimum - 1e-9 <= num <= s.maximum + 1e-9)):
                        flags.append("oor")
                    fault = is_fault_word(s) and not is_named(v) and int(v) != 0
                    if fault:
                        flags.append("oor")
                        msg_fault = True
                    if stale:
                        flags.append("stale")
                    raw = st.raws.get(s.name)
                    raw_txt = (f"raw {raw}" if s.length <= 4 else
                               f"raw 0x{raw & ((1 << s.length) - 1):0{(s.length + 3) // 4}X}"
                               ) if isinstance(raw, int) else ""
                    tag_s = self._colour_tag(key[0])[1]
                    self.tree.item(self.sig_iid(key, s.name),
                                   values=("", raw_txt, txt, short_unit(s.unit), "", "",
                                           "FAULT" if fault else
                                           ("OUT OF RANGE" if "oor" in flags else "")),
                                   tags=[tag_s] + flags)
            if msg_fault and status == "OK":
                status = "FAULT ACTIVE"
                self.tree.set(miid, "status", status)
            st.dirty = False

        sel = self.tree.selection()
        if sel and not sel[0].startswith("n:"):
            self.show_detail()

    def render_trace(self):
        if self.paused or not self.trace_pending:
            return
        n = min(TRACE_ROWS_PER_TICK, len(self.trace_pending))
        # newest frames matter most: skip ahead if we're behind
        while len(self.trace_pending) > n:
            self.trace_pending.popleft()
        for _ in range(n):
            ts, key, data, dlc = self.trace_pending.popleft()
            st = self.states.get(key)
            if not st or not self._matches(key):
                continue
            if st.dbmsg:
                try:
                    vals = st.dbmsg.decode(data, decode_choices=True, allow_truncated=True)
                    sigs = {s.name: s for s in st.dbmsg.signals}
                    dec = "  ".join(f"{k.replace('VCU_INV_', '').replace('INV_', '')}="
                                    f"{self.format_signal(sigs[k], v).split('  ')[0]}"
                                    f"{(' ' + short_unit(sigs[k].unit)) if short_unit(sigs[k].unit) else ''}"
                                    for k, v in vals.items())
                except Exception as e:
                    dec = f"decode error: {e}"
                name, tag = st.dbmsg.name, (self._colour_tag(key[0])[0],)
            else:
                name, dec, tag = "(unknown)", "", ("unknown",)
            t_txt = time.strftime("%H:%M:%S", time.localtime(ts)) + f".{int(ts * 1000) % 1000:03d}"
            self.trace.insert("", tk.END, values=(t_txt, fmt_id(*key), name, dlc,
                                                  hex_bytes(data), dec), tags=tag)
        kids = self.trace.get_children()
        if len(kids) > TRACE_MAX_ROWS:
            self.trace.delete(*kids[:len(kids) - TRACE_MAX_ROWS])
        if self.autoscroll.get():
            self.trace.yview_moveto(1.0)

    def update_status(self, now=None):
        now = now or time.time()
        while self.rate_window and now - self.rate_window[0][0] > 1.0:
            self.rate_window.popleft()
        rate = len(self.rate_window)
        bits = sum(b for _, b in self.rate_window)
        try:
            load = 100.0 * bits / int(self.rate_var.get())
        except ValueError:
            load = 0.0
        files = ", ".join(os.path.basename(f) for f in self.dbc.files) or "none"
        src = getattr(self, "source_label", "-") if self.source else "disconnected"
        unknown = sum(1 for s in self.states.values() if not s.dbmsg)
        parts = [src, f"{rate} msg/s", f"bus load ~{load:.0f}%",
                 f"{len(self.states)} IDs ({unknown} unknown)",
                 f"err frames {self.err_frames}", f"DBC: {files}"]
        if self.paused:
            parts.insert(0, "PAUSED")
        if self.logger:
            parts.insert(0, "REC")
        self.status_var.set("   |   ".join(parts))

    # ---------------------------------------------------------------- detail pane
    def _set_detail(self, chunks):
        top = self.detail.yview()[0]          # keep scroll position on refresh
        self.detail.configure(state=tk.NORMAL)
        self.detail.delete("1.0", tk.END)
        for text, tag in chunks:
            if tag:
                self.detail.insert(tk.END, text, tag)
            else:
                self.detail.insert(tk.END, text)
        self.detail.configure(state=tk.DISABLED)
        self.detail.yview_moveto(top)

    def show_detail(self):
        sel = self.tree.selection()
        if not sel:
            return
        iid = sel[0]
        parts = iid.split(":", 3)
        if parts[0] == "n":
            return
        key = (int(parts[1]), bool(int(parts[2])))
        st = self.states.get(key)
        if not st:
            return
        out = []
        h = lambda t: out.append((t, "h"))
        p = lambda t, tag=None: out.append((t, tag))

        if parts[0] == "m" or not st.dbmsg:
            m = st.dbmsg
            h(f"{m.name if m else 'Unknown frame'}\n")
            p(f"ID        {fmt_id(*key)}  ({key[0]} dec, "
              f"{'extended 29-bit' if key[1] else 'standard 11-bit'})\n")
            p(f"Sender    {st.node}\n")
            if m:
                p(f"Length    {m.length} bytes   (last DLC {st.dlc})\n")
                p(f"Cycle     {m.cycle_time or '-'} ms (DBC)\n")
                p(f"Signals   {len(m.signals)}"
                  f"{'  (multiplexed)' if m.is_multiplexed() else ''}\n")
                if m.comment:
                    p(f"\n{m.comment}\n")
            p(f"\nCount     {st.count}\nData      {hex_bytes(st.data)}\n")
            if st.error:
                p(f"\nDecode error: {st.error}\n")
            self._set_detail(out)
            return

        s = st.dbmsg.get_signal_by_name(parts[3])
        h(f"{s.name}\n")
        p(f"in {st.dbmsg.name} ({fmt_id(*key)})\n\n")
        v = st.values.get(s.name)
        if v is not None:
            h("Value     ")
            p(f"{self.format_signal(s, v)} {short_unit(s.unit)}\n")
            if s.name in st.raws:
                p(f"Raw       {st.raws[s.name]}\n")
        p(f"\nStart bit {s.start}   length {s.length}\n")
        p(f"Order     {'Intel (little endian)' if s.byte_order == 'little_endian' else 'Motorola (big endian)'}\n")
        p(f"Type      {'signed' if s.is_signed else 'unsigned'}"
          f"{' float' if s.is_float else ''}\n")
        p(f"Scale     {s.scale}   offset {s.offset}\n")
        p(f"Formula   physical = raw x {s.scale} + {s.offset}\n")
        p(f"Range     [{s.minimum} .. {s.maximum}] {s.unit or ''}\n")
        if s.multiplexer_ids:
            p(f"Mux page  {s.multiplexer_ids}\n")
        if s.comment:
            h("\nDescription\n")
            p(f"{s.comment.strip()}\n")
        if s.choices:
            h("\nValue table\n")
            cur = v.value if is_named(v) else None
            for k, name in sorted(s.choices.items()):
                p(f"  {k:>3} = {name}\n", "on" if k == cur else None)
        if is_fault_word(s) and v is not None:
            h("\nActive fault bits\n")
            names = self.dbc.fault_names(s.name, int(v))
            for n in names or ["  none"]:
                p(f"  {n}\n", "on" if names else None)
            if names and not self.dbc.fault_bits:
                p("\n  Add bit names to fault_bits.json\n  (from the inverter manual).\n")
        # bit map
        h("\nBit layout (byte rows, bit 7..0)\n")
        bits = signal_bit_positions(s)
        for byte in range(st.dbmsg.length):
            row = "".join(" #" if byte * 8 + b in bits else " ." for b in range(7, -1, -1))
            p(f"  B{byte} {row}\n")
        self._set_detail(out)

    # ---------------------------------------------------------------- shutdown
    def on_close(self):
        self.stop_source()
        if self.logger:
            self.logger.stop()
            self.logger = None
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CanViewerApp(root)
    # Optional: python can_viewer.py file1.dbc [file2.dbc ...]
    for p in sys.argv[1:]:
        try:
            app.dbc.load(p)
        except Exception as e:
            print(f"Could not load {p}: {e}")
    root.mainloop()


if __name__ == "__main__":
    main()
