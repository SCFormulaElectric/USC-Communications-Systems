# DX-LR02 framing test: how UART bytes become radio packets

Measured 2026-11-05 on the bench (two modules side by side, same laptop). Scripts and raw
logs are in this folder; see "How to rerun" at the end.

## Summary

- **The module does not use UART writes or idle gaps to decide where a radio packet
  ends. It uses a fixed window of about 101 ms that starts at the first byte it receives.**
  Everything that arrives in that window becomes one radio packet (up to 247 data bytes).
  This held for pauses of 0–94 ms, and for continuous 8-byte writes every 20 ms.
- Because of this, **the planned pattern (one 64 B packet per write on a 50 ms grid) loses
  81–84% of packets** (T3: 676 of 3600 received; T4: 261 of 1680). Packets 50 ms apart get
  merged. The window then closes in the middle of the next packet. The cut-off remainder is
  read as a new packet whose first 3 bytes are taken as address and channel, so it is
  thrown away along with the packets written after it.
- **Fix, verified with 0 loss:** never start a UART write sooner than ~120 ms after the
  previous one. Batch 2–3 telemetry packets into one write (≤ 250 B including the 3-byte
  header). 12/s and 14/s both ran with 0 loss this way (T8).
- **Limit:** one radio packet carries at most **247 data bytes** (250 B UART write with
  header). Bytes past that are lost. When the leftover is 3 B or more, its first 3 bytes are
  read as a header (see the next point).
- The RX module adds nothing: no prefix, RSSI, length, CR/LF or status text. The TX module
  prints nothing for good packets. It prints `channel Error:NN` when a packet's "channel"
  byte is 31 or higher. It accepted at least one fragment whose channel byte was 27
  (presumably transmitted on another channel; see "Unexpected").

## Setup and configuration

| Item | TX (car side) | RX (PC side) |
|---|---|---|
| Port | COM8 | COM9 |
| Adapter | CH340, USB VID:PID 1A86:7523, location 1-1 | CH340, 1A86:7523, location 1-2 |
| UART | 115200 8N1 (`+BAUD=8`) | 115200 8N1 (`+BAUD=8`) |
| Firmware | V2.2.0 | V2.2.0 |
| MODE | 1 (fixed-point) | 1 |
| LEVEL | 7 (13020.83 bps) | 7 |
| Spreading factor / bandwidth / CR | 5 / 0 / 2 | 5 / 0 / 2 |
| CRC | 0 (false) | 0 (false) |
| Frequency / channel | 437.2 MHz / 03 | 437.2 MHz / 03 |
| MAC | 0a,02 | 0a,01 |
| Power | 22 dBm | 22 dBm |
| Other | SLEEP 2, Preamble 8, IQ 0 | same |

- Configuration was read with `+++` → `AT+HELP` → `AT+BAUD` → `+++`. No settings were
  changed at any point. `AT+SF12` was never sent.
- The configuration was identical before testing, after the power cycle and after the
  soft reset (`logs/config_*.txt`).
- The ports had changed from the earlier test (COM11/COM5). COM8 was confirmed as TX by its
  MAC.
- **The adapters are CH340, not FTDI**, so there is no latency-timer setting. Measured
  read-to-read gaps inside one 64 B arrival were 3.0–3.6 ms (T1). The largest seen anywhere
  was 5.2 ms (T5).
- I grouped reads into arrival groups with a **10 ms gap threshold**. Real radio packets
  are always more than 100 ms apart, because each one needs its own window.
- **MODE1 header:** every write that starts a packet begins with `0A 01 03` (RX address +
  channel). The RX module removes it. Sizes below are data bytes, so a "64 B packet" is a
  67 B UART write.
- **Latency** = estimated time the last byte left the TX adapter (write start +
  bytes × 10 / 115200, or the write() return if later) → time of the PC read that delivered
  the last byte on RX. It includes the RX adapter's USB delay.

## Results table

| Question | Answer | Evidence (test, numbers) |
|---|---|---|
| One write = one radio packet? | **Yes, for one isolated write of up to 247 data bytes**, with nothing else written within ~101 ms of its first byte. Writes closer together than that are merged into one radio packet. | T1: every write of 8–247 B arrived as exactly one group with identical bytes (5/5 per size; 242–247 B 3/3). T5: two writes 0–50 ms apart arrived as one 131 B packet in 80/80 pairs. |
| Idle threshold that splits a packet | **There is no idle threshold.** The packet closes ~101 ms (100–102.5 ms) after its first byte, whether or not the UART is still busy. | T2: 32+32 B with pauses 0–94 ms → one exact 64 B packet (180/180). Pause 98 ms → cut inside the second chunk (35, 36, 48, 32… B received, 10/10). Pauses 100–200 ms → first 32 B only. Latency from the 2nd write fell 1:1 with pause (152 → 72 ms at 0 → 80 ms): the deadline is fixed from the first byte. T7: 8 B writes every 20 ms (idle gaps ~19 ms) → writes 0–5 (0–100 ms) formed one packet; the rest became new "packets" → `channel Error` (5/5 reps). |
| Max radio packet size / what happens above it | **247 data bytes** (250 B UART write). Above that, the first 247 B arrive. The leftover is dropped silently if under 3 B. Otherwise it is read as a new packet with its first 3 bytes as header: `channel Error:NN` if byte 3 ≥ 31, otherwise apparently sent to that channel. | T1 edge: 247 B exact (3/3); 248, 249 B → first 247 B, no message; 250, 255, 256, 300 B → 247 B + `channel Error:NN`, NN = data byte 249 (checked against the pattern). 512 B → 247 B + 2 errors. 256 B rep 2 (byte = 27): no error, nothing received. |
| Extra bytes or text added by RX module | **None.** Data only; MODE1 header removed. In merged packets, the *later* writes' headers (`0A 01 03`) come through as data. | T1: 60 single writes ≤ 247 B all byte-exact. T5: 131 B = 64 B + `0a0103` + 64 B (80/80). T6 and T8: no extra bytes. Idle 10 s: nothing printed (T0). |
| Echo/status text on TX module UART | **Nothing for good packets.** `channel Error:NN\r\n` per rejected packet (NN = decimal channel byte, all ≥ 31). `Power on\r\n` ~455 ms after `AT+RESET`. `Entry AT`/`Exit AT`/`OK` only in AT mode. | T1/T2/T5/T6/T8 (good-packet configs): 0 bytes on TX port. 2366 `channel Error` lines across all logs, smallest NN = 31. T0 reset log: `OK` at 3.107 s, `Power on` at 3.562 s. |
| 50 ms back-to-back packets merge or split? | **They merge, and the next packet usually gets cut.** Two 64 B packets ≤ 50 ms apart become one 131 B radio packet. A packet starting ~100 ms after a window opened is split across two windows and lost. | T5: 0/5/20/50 ms → one 131 B packet, 80/80. T3 timeline: e.g. seq 11 + first 20 B of seq 12 went out together; the rest of seq 12 → `channel Error:179`. |
| T3 results (good/bad CRC/lost/resync bytes) | **Fails: 18.8% delivered.** | 3600 sent over 300 s at 12/s: 676 good CRC, **2924 lost** (2923 by seq gaps), 0 duplicates, 0 corrupted packets passing CRC, 596 bad-CRC candidates (fragments), 12,220 of 55,484 RX bytes skipped while re-syncing, 614 of ~650 arrival groups not 64 B (mostly 82–85 B = one packet + header + fragment). Only 2.06 radio packets/s came out. TX printed 1571 `channel Error` lines. Latency of the good packets: p50 165 / p95 209 / max 237 ms. |
| T4 results | **Fails: 15.5% delivered.** | 1680 sent over 120 s at 14/s: 261 good, **1419 lost**, 0 duplicates, 240 bad-CRC candidates, 4638 of 21,342 bytes skipped, 247 groups not 64 B, 2.08 radio packets/s, 617 `channel Error`. Latency p50 165 / p95 170 / max 236 ms. |
| 64 B latency p50/p95/max | **152.2 / 152.4 / 152.8 ms** (single 64 B packet per write, writes ≥ 120 ms apart, n = 479). | T8 (1 packet per write, 120/150/200/250 ms): p50 152.15, p95 152.34–152.37, max 152.79 ms. T1: 152.4–153.2 ms (n = 5). With 3 packets per write: p50 226.5 / p95 232.4 ms; 2 per write: 188.3 / 193.5 ms. |
| Anything unexpected | See below. Most important: possible transmissions on other channels, and a spontaneous RX reboot. | — |

### Latency vs. size (T1, single write, p50 of 5, first byte / last byte, ms)

| Data bytes | 8 | 32 | 64 | 100 | 128 | 200 | 239–241 |
|---|---|---|---|---|---|---|---|
| First / last byte | 119 / 119 | 133 / 133 | 150 / 153 | 170 / 177 | 185 / 193 | 224 / 240 | 245 / 265 |

Last-byte latency is roughly the ~101 ms window, plus airtime (~0.6 ms per byte at LEVEL 7),
plus the RX module's UART output.

## Extra tests (not in the task, added to find what works)

**T6 – `+++` inside packet data.** 64 B packets containing `+++` at byte 30, `+++\r\n` at
byte 30, or `+++` at bytes 61–63: all 9 arrived byte-exact. The TX module stayed in data
mode (checked with `+++` → `Entry AT` afterwards). Data containing `+++` is safe.

**T7 – trickle.** See the idle-threshold row above.

**T8 – rate and batching, production 64 B packets.** One header per write, k packets per
write:

| k per write | Write period | Packets/s | Sent | Received | Loss | Latency p50 / p95 / max ms | `channel Error` |
|---|---|---|---|---|---|---|---|
| 1 | 100 ms | 10.0 | 200 | 40 | **80%** | 164.7 / 164.9 / 166.2 | 111 |
| 1 | 120 ms | 8.3 | 166 | 166 | 0% | 152.1 / 152.4 / 152.8 | 0 |
| 1 | 150 ms | 6.7 | 133 | 133 | 0% | 152.2 / 152.4 / 152.8 | 0 |
| 1 | 200 ms | 5.0 | 100 | 100 | 0% | 152.2 / 152.3 / 152.7 | 0 |
| 1 | 250 ms | 4.0 | 80 | 80 | 0% | 152.2 / 152.3 / 152.5 | 0 |
| 3 | 250 ms | **12.0** | 240 | 240 | **0%** | 226.6 / 232.5 / 249.3 | 0 |
| 3 | 300 ms | 10.0 | 198 | 198 | 0% | 226.5 / 232.4 / 232.6 | 0 |
| 3 | 400 ms | 7.5 | 150 | 150 | 0% | 226.6 / 232.4 / 233.7 | 0 |
| 3 | 214 ms | **14.0** | 420 | 420 | **0%** | 226.5 / 232.4 / 233.4 | 0 |
| 2 | 143 ms | **14.0** | 418 | 418 | **0%** | 188.3 / 193.5 / 194.3 | 0 |

- 20 s per row; 30 s for the two 14/s rows.
- In every 0%-loss row the RX stream was clean: 0 bytes skipped, 0 bad-CRC candidates, and
  arrivals were exactly k × 64 B.
- In the 214 ms and 143 ms rows, a new write starts before the previous batch has finished
  arriving, so the module buffers new input while it is transmitting.

## What this means for the car firmware

1. **Pace UART writes, not packets.** Start each write at least ~120 ms after the previous
   one started; 100 ms fails. Keep the whole write well within the ~101 ms window (250 B
   takes 21.7 ms at 115200, so that's fine).
2. **For 12–14 packets/s, batch.** Put 2–3 complete 64 B packets behind one `0A 01 03`
   header in a single write. Verified: 3 every 250 ms (12/s), 3 every 214 ms (14/s), 2 every
   143 ms (14/s), all with 0 loss. Four packets (259 B) would exceed the 250 B limit. Two
   per write has lower latency (188 ms vs 227 ms p50).
3. **Never write more than 250 B** (header included) in one window. The excess is lost and
   may go out on another channel.
4. The stream parser plan (sync `A5 5A` + length 64 + CRC) is right. With correct pacing,
   packets arrive back to back with no bytes in between.
5. The STM32 should ignore `Power on\r\n` at module boot. It could count `channel Error`
   lines on the TX UART as a health signal: zero means the framing is correct.

## Unexpected

- **Fragments may be transmitted on other channels.**
  - Of 2366 rejected fragments, the smallest rejected channel value was 31. One fragment
    with channel byte 27 (T1, 256 B rep 2) produced no error and no RX output.
  - So a cut-off fragment whose third byte is 0–30 (about 12% of random data) is probably
    sent as a stray radio packet to a random address on channel 0–30. This would affect
    other users of that band.
  - **Not verified over the air**: the RX module only listens on channel 03.
- **The RX module rebooted once without being touched by the scripts.** During the config
  query right after the TX power cycle, COM9 answered `+++` with `Power on`. It was fine
  afterwards and its configuration was unchanged. Cause unknown; it may have been bumped
  while the TX adapter was replugged.
- The 1-packet-per-write, 100 ms case delivers 20%, not 0%. The ~1 ms by which the window
  outlasts the write period accumulates until a write lands cleanly in a fresh window.

## Not measured / deviations

- **Power cycle (T0):** the TX module was power-cycled by unplugging its USB adapter. COM8
  was already back when logging started, so **the TX module's startup output after a real
  power cycle was not captured**. Substitute: logged a soft reboot (`+++`, `AT+RESET`):
  `Entry AT`, `OK`, then `Power on\r\n` 455 ms later; the RX port printed nothing. Idle
  10 s: nothing on either port. To capture real power-up text, keep the USB adapter
  plugged in and disconnect only the module's VCC while `powercycle_log.py` runs.
- **The exact window start** (first byte arriving at the module vs. at the adapter) can't
  be separated from USB timing (~±1 ms). Hence "100–102.5 ms".
- The valid channel range (0–30?) is inferred from which values errored. It was not
  checked against a datasheet.
- Not tested: write periods between 100 and 120 ms; more than 3 packets per write at
  shorter periods; range or RF conditions (modules were side by side); other LEVELs.
- Spec deviations:
  - T1 used 2 s spacing (not 1 s) for writes above 200 B.
  - T2 added pauses of 60, 80, 100, 150 and 200 ms plus a fine sweep (82–98 ms), because
    the boundary was above 50 ms.
  - T6–T8 are extra tests.
  - All traffic stayed at ≤ 14 packets/s except the short T1, T2, T5 and T7 runs.

## How to rerun

Python 3 + pyserial. From this folder (TX first, RX second):

```
py query_config.py COM8 COM9 logs/config.txt     # read-only config dump
py framing_tests.py run t0 --tx COM8 --rx COM9   # idle log (10 s)
py reset_log.py COM8 COM9                        # soft-reset startup text
py powercycle_log.py COM8 COM9 120               # log while you power-cycle TX
py framing_tests.py run t1                       # sizes (add --sizes 242,247,248 --name t1_edge)
py framing_tests.py run t2 --pauses 0,0.5,1,2,3,5,8,10,15,20,30,50,60,80,100,150,200
py framing_tests.py run t5                       # back-to-back pairs
py framing_tests.py run t3                       # 12/s, 5 min
py framing_tests.py run t4                       # 14/s, 2 min
py framing_tests.py run t6                       # '+++' in data
py framing_tests.py run t7                       # trickle
py framing_tests.py run t8                       # rate/batching (--configs 3x214,2x143 --duration 30)
py framing_tests.py run t9                       # PC -> car commands: t9a-t9d in a row (~11 min)
py framing_tests.py run t9b --duration 60        # one T9 sub-test, shorter
py framing_tests.py analyze t3 --gap-ms 10       # re-analyze any saved log
```

- Defaults: `--tx COM8 --rx COM9`.
- Raw logs: `logs/<test>_raw.csv`, one row per read or write: `t_s, port, nbytes, hex,
  t_write_start_s, tag`. Summaries: `logs/<test>_summary.txt` and `.json`.
- `blink.py COM8` flickers an adapter's LEDs with AT queries so you can find it.
