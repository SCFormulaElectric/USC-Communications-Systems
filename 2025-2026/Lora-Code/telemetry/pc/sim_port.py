"""A simulated car on a fake serial port, for the GUI's --demo mode and the tests.

Looks like a pyserial port to LoraLink: every 200 ms it produces a batch of 3 x 64 B packets in
the firmware's slot order (FAST, FAST, then THERM / DIAG / EVENT / DIAG_RESP by write index).
Values vary over time. Commands written to it are acknowledged like the car does (FAST bytes
52-53 and DIAG_RESP), and BAMO_READ gets a Bamocar reply frame on 0x181.
"""
import math
import random
import threading
import time

from decoder import Schema


class SimPort:
    def __init__(self, schema=None, *args, **kwargs):
        self.s = schema or Schema()
        self.lock = threading.Lock()
        self.t0 = time.perf_counter()
        self.next_batch = self.t0 + 0.2
        self.buf = bytearray()
        self.seq = 0
        self.n = 0
        self.ack = (0, 0xFF)
        self.acks = []
        self.frames = []
        self.events = []
        self.regs = {0xC8: 6000, 0x25: 1000, 0x4D: 400, 0xC4: 600, 0xC5: 300, 0xC6: 500, 0xD9: 1000,
                     0x35: 500, 0x8F: 0}
        self.rnd = random.Random(7)
        self.r2d = 0
        self.last_event_t = self.t0
        self.is_open = True

    # ------------------------------------------------------------------ values
    def _fast(self, t):
        ph = t - self.t0
        drive = (ph % 40) > 10                          # 30 s driving, 10 s idle
        if drive != bool(self.r2d):
            self.r2d = int(drive)
            self.events.append((t, 2 if drive else 3, 0))
        sp = int(16384 * max(0.0, math.sin(ph / 3))) if drive else 0
        tq = int(16384 * max(0.0, math.sin(ph / 2))) if drive else 0
        valid = 0xFFFF & ~(1 << 7) & ~(1 << 8)          # no KERN_STATUS / DC bus (GEVCU change pending)
        if not drive:
            valid &= ~(1 << 11)                         # 0x473 only while driving
        status = (1 << 1) | (self.r2d << 4) | (self.r2d << 5)
        return {"valid": valid, "status": status,
                "brake1": 256 + int(50 * math.sin(ph)), "brake1_prev": 256,
                "brake2": 258 + int(50 * math.sin(ph)), "brake2_prev": 258,
                "torque_cmd": tq, "torque_cmd_prev": tq, "torque_req": tq // 100 if drive else 0,
                "speed": sp, "dc_bus": 0, "pack_current": 100 + tq // 200, "pack_voltage": 3456 - tq // 500,
                "pack_soc": 85, "max_cell_temp": 32 + int(ph / 60) % 5, "bms_fault_num": 0,
                "motor_temp": 9369 + int(55.21 * (40 + 5 * math.sin(ph / 30))), "igbt_temp": 12000,
                "i_actual": tq // 30, "kern_status": 0, "bms_fault_flags": 0, "pack_ocv": 3472,
                "pack_resistance": 42, "bms_high_temp": 45, "bms_low_temp": 25, "soc16": 8500,
                "can_frames_100ms": 38 + self.rnd.randint(0, 4), "ack_seq": self.ack[0], "ack_status": self.ack[1]}

    def _therm(self, t):
        mt = 40 + int(5 * math.sin((t - self.t0) / 30))
        return {"f380": bytes([1, 2, 32, 4, 5, 6, 7, 8]), "f381": bytes(range(8)), "f382": bytes(range(8, 16)),
                "f383": bytes(range(16, 24)), "f384": bytes(range(24, 32)), "f385": bytes(range(32, 40)),
                "seen": 0x7F, "motor_temp_vcu": mt.to_bytes(2, "big", signed=True)}

    def _diag(self, t):
        ph = t - self.t0
        return {"fw_version": 0x0100, "build_flag": 1, "build_opts": 1, "can_rx_total": int(ph * 400),
                "can_rx_unused": int(ph * 3), "nmax": self.regs[0xC8], "ramp": self.regs[0x25],
                "i_max": self.regs[0x4D], "i_max_pk": self.regs[0xC4], "i_con_eff": self.regs[0xC5],
                "i_device": self.regs[0xC6], "i_200pc": self.regs[0xD9], "reg_seen": 0x7F,
                "raw_481": 0x3412, "raw_666": 0x7856, "raw_321": 0, "raw_seen": 3}

    def _event(self, t, up):
        recs = []
        while self.events and len(recs) < 10:
            et, code, val = self.events.pop(0)
            recs.append({"age_ms": max(0, min(65535, int((t - et) * 1000))), "code": code, "value": val})
        return {"count": len(recs), "records": recs}

    def _diag_resp(self, t):
        acks, self.acks = self.acks[:3], self.acks[3:]
        frames, self.frames = self.frames[:3], self.frames[3:]
        return {"n_acks": len(acks), "acks": [{"seq": a, "status": s} for a, s in acks],
                "n_frames": len(frames), "frames": frames}

    # ------------------------------------------------------------------ batches
    def _pump(self):
        now = time.perf_counter()
        while now >= self.next_batch:
            t = self.next_batch
            up = int((t - self.t0) * 1000) + 1000
            if t - self.last_event_t > 7:
                self.events.append((t, 1, self.rnd.randint(1, 4)))     # dash button now and then
                self.last_event_t = t
            pk = [("FAST", self._fast(t - 0.1), up - 100), ("FAST", self._fast(t), up)]
            k = self.n % 5
            if k == 0:
                pk.append(("THERM", self._therm(t), up))
            elif k == 1:
                pk.append(("DIAG", self._diag(t), up))
            elif self.events:
                pk.append(("EVENT", self._event(t, up), up))
            elif self.acks or self.frames:
                pk.append(("DIAG_RESP", self._diag_resp(t), up))
            else:
                pk.append(("DIAG", self._diag(t), up))
            for name, fields, u in pk:
                self.buf += self.s.encode_packet(name, fields, seq=self.seq, uptime_ms=u)
                self.seq = (self.seq + 1) & 0xFF
            self.n += 1
            self.next_batch += 0.2

    # ------------------------------------------------------------------ pyserial interface
    @property
    def in_waiting(self):
        with self.lock:
            self._pump()
            return len(self.buf)

    def read(self, n=1):
        with self.lock:
            self._pump()
            if not self.buf:
                time.sleep(0.002)
                return b""
            out = bytes(self.buf[:n])
            del self.buf[:n]
            return out

    def write(self, data):
        with self.lock:
            if data[:3] == self.s.header_to_car and len(data) >= 27:
                c = self.s.decode_command(data[3:27])
                if c["crc_ok"]:
                    st = 0
                    a = c["args"]
                    if c["code_name"] == "BAMO_READ" and a["period"] != 0xFF:
                        v = self.regs.get(a["reg"], 0)
                        self.frames.append({"age_ms": 5, "id": 0x181, "dlc": 4,
                                            "data": bytes([a["reg"], v & 0xFF, (v >> 8) & 0xFF, 0, 0, 0, 0, 0])})
                    elif c["code_name"] == "BAMO_WRITE":
                        if a["reg"] in (0x90, 0x31, 0x6E, 0x51, 0x68, 0x84, 0xDC):
                            st = 4                                          # REJ_REGID
                        elif self.r2d:
                            st = 5                                          # REJ_INV_ENABLED
                        else:
                            self.regs[a["reg"]] = a["value"] & 0xFFFF
                    self.ack = (c["seq"], st)
                    self.acks.append((c["seq"], st))
        return len(data)

    def close(self):
        self.is_open = False
