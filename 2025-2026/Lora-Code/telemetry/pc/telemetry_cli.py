#!/usr/bin/env python3
"""Live telemetry + Bamocar diagnostics console (BUILD_INSTRUCTIONS.md 7.5).

    py telemetry_cli.py --port COM9 [--delay-ms 110] [--log-dir logs] [--watch]

Status line twice a second: speed, torque command, pack V/I/SOC, max cell temp, brakes, status
bits, packets/s, loss %, link state. Type `help` for the diagnostics commands. `--watch` shows
only the status line (no prompt). Logs go to <log-dir>/raw_<stamp>.bin and <log-dir>/<stamp>/*.csv.
"""
import argparse
import os
import shlex
import sys
import threading
import time

from decoder import EXT_FLAG, Schema
from logger import TelemetryLogger
from lora_link import LoraLink

HELP = """commands (register: number like 0xC8 or a name like N_MAX):
  ping                                    start / keep a diagnostics session
  read <reg> [--every <ms>]               read once, or repeat every 20-254 ms
  stop <reg>                              stop a repeating read
  write <reg> <value> [--32]              Bamocar parameter write (16-bit, or 32-bit with --32)
  raw <id> <bytes...> [--ext]             send a CAN frame (only 0x201 unless DIAG_ANY_ID=1)
  capture <id> [<mask>] [--slot N] [--rate N] [--ext]   forward matching CAN frames
  uncapture [<slot>]                      clear one capture slot (default: all)
  stopall                                 stop reads, clear captures, end the session
  status                                  link, command and car counters
  quit"""

STATUS_WORDS = {
    "OK": "OK", "REJ_DISABLED": "rejected: this build doesn't allow it",
    "REJ_ID": "rejected: CAN ID not allowed", "REJ_REGID": "rejected: register blocked",
    "REJ_INV_ENABLED": "rejected: inverter may be live", "REJ_RATE": "rejected: rate limit",
    "REJ_LIMIT": "rejected: too many repeating reads (max 3)", "BAD_ARGS": "rejected: bad arguments",
    "TX_FAIL": "failed: CAN send failed (bus-off?)",
}


def enable_vt():
    if not sys.stdout.isatty():
        return False
    if os.name != "nt":
        return True
    try:
        import ctypes
        k = ctypes.windll.kernel32
        h = k.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if k.GetConsoleMode(h, ctypes.byref(mode)):
            return bool(k.SetConsoleMode(h, mode.value | 0x0004))
    except Exception:
        pass
    return False


class Cli:
    def __init__(self, link, schema, watch=False):
        self.link, self.s = link, schema
        self.watch = watch
        self.vt = enable_vt()
        self.out_lock = threading.Lock()
        self.wanted_regs = set()          # registers the user asked for: show their replies
        self.running = True
        link.listeners.append(self.on_event)

    # ------------------------------------------------------------------ output
    def status_line(self):
        st = self.link.stats()
        fast = self.link.latest.get("FAST")
        parts = [f"{st['state']:<8} {st['pps']:4.1f} pkt/s loss {st['loss_pct']:4.1f}%"]
        if fast:
            v, f = fast["values"], fast["fields"]
            rpm = v.get("speed")
            parts.append(f"{rpm:6.0f} rpm" if rpm is not None else f"speed raw {f['speed']}")
            parts.append(f"tq {v['torque_cmd']:5.1f}%")
            parts.append(f"pack V {f['pack_voltage']} I {f['pack_current']} SOC {f['pack_soc']}")
            parts.append(f"cell {f['max_cell_temp']}C")
            parts.append(f"brk {f['brake1']}/{f['brake2']}")
            parts.append(" ".join(v["status"]) or "-")
            if len(v["valid"]) < 16:
                parts.append(f"valid 0x{f['valid']:04X}")
        if st["session"]:
            parts.append("session")
        return " | ".join(parts)

    def say(self, msg):
        with self.out_lock:
            if self.vt and not self.watch:
                sys.stdout.write("\x1b[1A\r\x1b[2K" + msg + "\n\x1b[2K" + self.status_line() + "\n\r\x1b[2K> ")
            else:
                sys.stdout.write(msg + "\n")
            sys.stdout.flush()

    def status_loop(self):
        while self.running:
            time.sleep(0.5)
            with self.out_lock:
                if self.watch:
                    sys.stdout.write("\r" + self.status_line()[:150].ljust(150))
                elif self.vt:
                    sys.stdout.write("\x1b7\x1b[1A\r\x1b[2K" + self.status_line() + "\x1b8")
                sys.stdout.flush()

    # ------------------------------------------------------------------ events
    def on_event(self, ev):
        k = ev["kind"]
        if k == "result":
            c, st = ev["cmd"], ev["status"]
            if c.get("note") == "keepalive" and st == 0:
                return
            what = self.describe(c)
            if st is None:
                self.say(f"{what} -> FAILED: no acknowledgement after {c['tries']} tries")
            else:
                name = self.s.status_name(st)
                self.say(f"{what} -> {STATUS_WORDS.get(name, name)}")
        elif k == "state":
            self.say(f"link: {ev['text']}")
        elif k == "power_on":
            self.say("PC LoRa module printed 'Power on'")
        elif k == "channel_error":
            self.say(f"PC LoRa module: {ev['text']}")
        elif k == "packet" and ev["packet"]["type_name"] == "DIAG_RESP":
            f = ev["packet"]["fields"]
            for fr in f["frames"][:f["n_frames"]]:
                ext = bool(fr["id"] & EXT_FLAG)
                fid = fr["id"] & 0x1FFFFFFF
                b = None if ext else self.s.bamo_frame({"id": fid, "dlc": fr["dlc"], "data": fr["data"]})
                if b:
                    # 0x181 also carries the GEVCU's own repeating reads: show only what was asked for
                    if b["reg"] in self.wanted_regs:
                        self.say(f"  {b['name']} = {b['value']} (0x{b['value'] & 0xFFFFFFFF:X})")
                else:
                    self.say(f"  frame 0x{fid:X}{' ext' if ext else ''} [{fr['dlc']}] {fr['data'][:fr['dlc']].hex(' ')}")
            if f["cap_dropped"]:
                self.say(f"  ({f['cap_dropped']} captured frames dropped on the car)")
        elif k == "error":
            self.say(f"serial error: {ev['text']}")

    def describe(self, c):
        a, n = c["args"], c["code_name"]
        if n == "BAMO_READ":
            p = a["period"]
            how = "once" if p == 0 else ("stop" if p == 0xFF else f"every {p} ms")
            return f"read {self.s.reg_name(a['reg'])} {how}"
        if n == "BAMO_WRITE":
            return f"write {self.s.reg_name(a['reg'])} = {a['value']} ({a['size'] * 8}-bit)"
        if n == "SEND_FRAME":
            return f"raw 0x{a['id'] & 0x1FFFFFFF:X} {bytes(a['data'][:a['dlc']]).hex(' ')}"
        if n == "CAPTURE_SET":
            return f"capture slot {a['slot']} id 0x{a['id'] & 0x1FFFFFFF:X} mask 0x{a['mask']:X} rate {a['rate']}/s"
        if n == "CAPTURE_CLEAR":
            return f"uncapture {'all' if a['slot'] == 0xFF else a['slot']}"
        return n.lower()

    # ------------------------------------------------------------------ commands
    def submit(self, code, args=None, note=""):
        if not self.link.sender.session:
            # a new session: the car starts with no captures; forward 0x181 for Bamocar replies
            self.link.submit("CAPTURE_SET", {"slot": 0, "id": 0x181, "mask": 0x7FF, "rate": 20}, "auto")
        self.link.submit(code, args, note)

    def handle(self, line):
        try:
            tok = shlex.split(line)
        except ValueError as e:
            self.say(f"? {e}")
            return
        if not tok:
            return
        cmd, args = tok[0].lower(), tok[1:]

        def opt(name, default=None, conv=lambda x: int(x, 0)):
            if name in args:
                i = args.index(name)
                v = conv(args[i + 1])
                del args[i:i + 2]
                return v
            return default

        def flag(name):
            if name in args:
                args.remove(name)
                return True
            return False

        if cmd in ("quit", "exit"):
            self.running = False
        elif cmd == "help":
            self.say(HELP)
        elif cmd == "ping":
            self.submit("PING")
        elif cmd == "read":
            every = opt("--every")
            reg = self.s.parse_reg(args[0])
            if every is not None and not 20 <= every <= 254:
                self.say("--every must be 20-254 ms")
                return
            if reg == 0x8F:
                ans = input("WARNING: the GEVCU sends a Bamocar error clear when it sees the 0x8F reply with "
                            "byte 1 = 0x20. Read anyway? [y/N] ")
                if ans.strip().lower() != "y":
                    return
            self.wanted_regs.add(reg)
            self.submit("BAMO_READ", {"reg": reg, "period": every or 0})
        elif cmd == "stop":
            reg = self.s.parse_reg(args[0])
            self.submit("BAMO_READ", {"reg": reg, "period": 0xFF})
        elif cmd == "write":
            wide = flag("--32")
            reg, value = self.s.parse_reg(args[0]), int(args[1], 0)
            size = 4 if wide else 2
            self.submit("BAMO_WRITE", {"reg": reg, "size": size, "value": value & (0xFFFFFFFF if wide else 0xFFFF)})
        elif cmd == "raw":
            ext = flag("--ext")
            cid = int(args[0], 0)
            data = bytes.fromhex("".join(args[1:]))
            if len(data) > 8:
                self.say("at most 8 data bytes")
                return
            ext = ext or cid > 0x7FF
            self.submit("SEND_FRAME", {"id": cid | (EXT_FLAG if ext else 0), "dlc": len(data),
                                       "data": data.ljust(8, b"\0")})
        elif cmd == "capture":
            ext = flag("--ext")
            slot, rate = opt("--slot", 0), opt("--rate", 20)
            cid = int(args[0], 0)
            ext = ext or cid > 0x7FF
            mask = int(args[1], 0) if len(args) > 1 else (0x1FFFFFFF if ext else 0x7FF)
            self.submit("CAPTURE_SET", {"slot": slot, "id": cid | (EXT_FLAG if ext else 0), "mask": mask, "rate": rate})
        elif cmd == "uncapture":
            self.submit("CAPTURE_CLEAR", {"slot": int(args[0], 0) if args else 0xFF})
        elif cmd == "stopall":
            self.link.submit("STOP_ALL")
        elif cmd == "status":
            self.print_status()
        else:
            self.say(f"unknown command '{cmd}' (type help)")

    def print_status(self):
        st = self.link.stats()
        lines = [f"link: {st['state']}, {st['pps']:.1f} packets/s, loss {st['loss_pct']:.1f}% (30 s), "
                 f"received {st['received']}, missing {st['missing']}, bad CRC {st['bad_crc']}, "
                 f"'Power on' {st['power_on']}"]
        c = st["inflight"]
        lines.append(f"commands: session {'open' if st['session'] else 'closed'}, waiting: "
                     f"{self.describe(c) + ' (try ' + str(c['tries']) + ')' if c else 'none'}, queued {st['queued']}")
        d = self.link.latest.get("DIAG")
        if d:
            f = d["fields"]
            lines.append(f"car: fw 0x{f['fw_version']:04X} flag {f['build_flag']} opts {d['values']['build_opts']}, "
                         f"CAN rx {f['can_rx_total']} (unused {f['can_rx_unused']}, overflow {f['can_rx_overflow']}), "
                         f"TEC {f['can_tec']} REC {f['can_rec']} bus-off {f['can_busoff_count']}")
            lines.append(f"     LoRa 'channel Error' {f['lora_channel_errors']} 'Power on' {f['lora_power_on']}, "
                         f"commands bad {f['cmd_bad']} executed {f['cmd_executed']}, events dropped "
                         f"{f['events_dropped']}, capture dropped {f['capture_dropped']}")
        fast = self.link.latest.get("FAST")
        if fast:
            f = fast["fields"]
            lines.append(f"last ack: seq {f['ack_seq']} {self.s.status_name(f['ack_status'])}; "
                         f"valid {' '.join(fast['values']['valid'])}")
        self.say("\n".join(lines))

    # ------------------------------------------------------------------ main loop
    def run(self):
        threading.Thread(target=self.status_loop, daemon=True).start()
        if self.watch:
            try:
                while self.running:
                    time.sleep(0.2)
            except KeyboardInterrupt:
                pass
            return
        print("type 'help' for commands\n")
        while self.running:
            try:
                line = input("> ")
            except (EOFError, KeyboardInterrupt):
                break
            try:
                self.handle(line)
            except (IndexError, ValueError, KeyError) as e:
                self.say(f"? {e} (type help)")
        self.running = False
        if self.link.sender.session:
            print("closing the session (STOP_ALL) ...")
            self.link.submit("STOP_ALL")
            t_end = time.time() + 3
            while time.time() < t_end and self.link.sender.session:
                time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--delay-ms", type=int, default=110, help="command delay after a batch arrives (default 110)")
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--watch", action="store_true", help="status line only")
    a = ap.parse_args()
    schema = Schema()
    logger = None if a.no_log else TelemetryLogger(a.log_dir, schema)
    link = LoraLink(a.port, schema, delay_ms=a.delay_ms, logger=logger)
    link.start()
    if logger:
        print(f"logging to {logger.raw_path} and {logger.dir}")
    try:
        Cli(link, schema, watch=a.watch).run()
    finally:
        link.stop()
        if logger:
            logger.close()


if __name__ == "__main__":
    main()
