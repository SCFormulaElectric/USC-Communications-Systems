# Build task: car telemetry over LoRa, with Bamocar diagnostics

You are building a two-way telemetry link for SC Formula Electric's car:

- **Car side:** firmware for the existing LoRa board (STM32F042F4, project
  `2025-2026/Lora-Code/LoRa_board/`). It listens to the car's CAN bus, packs signals into
  fixed packets, and sends them through a DX-LR02 LoRa module. In a diagnostics build it
  also takes commands from the PC and talks to the Bamocar motor inverter.
- **PC side:** Python tools that receive, decode and log the telemetry, and a console for
  sending diagnostic commands.
- **Shared:** one schema file that defines every packet, so both sides agree.

Everything about the radio link below was **measured** on the bench. Don't change the
transport rules (section 2); they are the difference between 0% and 80% loss. The test
report is `2025-2026/Lora-Code/tools/lora_framing_test/lora_framing_results.md` (read it
before starting) with scripts and logs beside it.

Work on branch `lora-telemetry`. Put new files under `2025-2026/Lora-Code/telemetry/`
(schema, generator, PC tools, host tests) and the firmware in the `LoRa_board` project.

---

## 1. Hardware facts

**Car board: STM32F042F4** (16 KB flash, 6 KB RAM), from `LoRa_board.ioc` / `main.c`:

| Item | Setting |
|---|---|
| Clock | HSI 8 MHz × PLL 6 = 48 MHz (keep the existing `SystemClock_Config`) |
| CAN | PA11 RX / PA12 TX (pin remap already set), 500 kbit/s: prescaler 6, BS1 13 TQ, BS2 2 TQ |
| LoRa module UART | USART2, PA2 TX / PA3 RX. **Currently 9600 baud. Change to 115200 8N1.** |
| LED | PA5 |

Add DMA for USART2 TX (one-shot transfers) in CubeMX. For USART2 RX use either circular
DMA or the RXNE interrupt into a ring buffer. Check the DMA channel mapping for the F042 in
CubeMX rather than assuming it.

**Flash is the tightest constraint.** Build with `-Os`. Don't use `printf`, `snprintf`,
`sprintf`, floating point or `malloc`. If HAL makes it too big, switch USART/DMA to LL
drivers. Report `arm-none-eabi-size` output for both build flavours (section 6.1). If the
diagnostics build doesn't fit, say so and stop: don't remove safety checks to make it fit.

**LoRa modules (DX-LR02, firmware V2.2.0), already configured:**

| Setting | Car module | PC module |
|---|---|---|
| MODE | 1 (fixed-point) | 1 |
| LEVEL | 7 (SF5) | 7 |
| Channel | 03 (437.2 MHz) | 03 |
| MAC | 0a,02 | 0a,01 |
| Radio CRC | 0 (off) | 0 |
| UART | 115200 8N1 | 115200 8N1 |

**The firmware must not send any AT command.** The current `main.c` sends AT config at
boot, including `AT+SF12`, which drops the link to 244 bps. Delete all of it. The modules
are configured once by hand.

---

## 2. Radio transport rules (measured; don't change)

1. **Every UART write to a module starts with a 3-byte header:** destination MAC + channel.
   - Car → PC: `0A 01 03`. PC → car: `0A 02 03`. The receiving module strips it and adds
     nothing.
2. **The module packs everything that arrives within ~101 ms of a write's first byte into
   one radio packet.** There is no idle-gap rule. Max 247 data bytes (250 B write).
3. **Writes from one side must start at least 120 ms apart and be at most 250 B.**
   Breaking this cuts packets, and the cut-off piece can be transmitted on another channel.
4. **The car writes every 200 ms: header + exactly 3 × 64 B packets (195 B) as one DMA
   transfer.** Always 3 packets, never 2: the PC times its commands from when each batch
   arrives, and that only works if every batch is the same size. Verified: 4500/4500
   packets over 5 min.
5. **The modules are half-duplex.** If a command and a telemetry batch are on air at the
   same time, both are lost. The PC therefore sends a command **110 ms after the last byte
   of a telemetry batch arrives**. Verified: 1283/1283 commands, no telemetry lost.
6. **The PC module sometimes locks up** when both directions carry traffic (cause
   unknown). While locked, commands aren't delivered and about 1 in 5 telemetry batches is
   lost. It restarts itself after ~68.5 s and prints `Power on`. The PC software must
   survive this (section 7.4). The car module never locked up. Telemetry-only use never
   triggered it.
7. Text a module may print on its UART: `Power on\r\n` (after boot), `channel Error:NN\r\n`
   (a bad write). Both receivers must skip it. The car counts `channel Error` lines: the
   count must stay 0.

---

## 3. Packets

All multi-byte fields on the radio are **little-endian**. CRC is **CRC-16/CCITT-FALSE**
(poly 0x1021, init 0xFFFF, no reflection, no final XOR; `"123456789"` → `0x29B1`). In Python
that is `binascii.crc_hqx(data, 0xFFFF)`.

### 3.1 Telemetry packet (car → PC): 64 B

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Sync `A5 5A` |
| 2 | 1 | `version << 4 \| type`. Version = 1 |
| 3 | 1 | Packet sequence number (u8, one counter for all packets, wraps) |
| 4 | 4 | Car uptime in ms when the packet was built (u32) |
| 8 | 54 | Payload (by type, below). Unused bytes = 0 |
| 62 | 2 | CRC over bytes 0–61 |

Types: 1 FAST, 2 THERM, 3 DIAG, 4 EVENT, 5 DIAG_RESP. (Type 8 is the command, 3.7.)

**Which packets go in each write (every 200 ms):**
- Packet 1: FAST built 100 ms earlier. Packet 2: FAST built now. So FAST is 10 Hz.
- Packet 3, by write index `n = write_count % 5` (5 writes per second):
  - `n == 0`: THERM. `n == 1`: DIAG.
  - `n == 2..4`: EVENT if events are waiting, else DIAG_RESP if it has anything to send,
    else DIAG (filler; repeating DIAG is fine).
- **Diagnostics-only build (flag = 2):** packet 1 is FAST only when `n == 0`; otherwise
  packets 1–3 are filled in the order DIAG_RESP, EVENT, then THERM/DIAG as filler. Still
  exactly 3 packets per write.

### 3.2 FAST payload (type 1, 54 B)

Offsets are within the payload (add 8 for the packet offset). "CAN" is the source and its
byte order on the CAN bus; the STM32 decodes it to an integer and stores it LE.

| Off | Type | Field | CAN source |
|---|---|---|---|
| 0 | u16 | `valid` bitmap (3.3) | — |
| 2 | u16 | `status` bits (3.3) | — |
| 4 | u16 | brake1 now | 0x364 B0–1, BE |
| 6 | u16 | brake1 50 ms earlier | same |
| 8 | u16 | brake2 now | 0x365 B0–1, BE |
| 10 | u16 | brake2 50 ms earlier | same |
| 12 | i16 | Inverter torque command now (0x7FFF = 100%) | 0x201 with B0 = 0x90: B1–2, LE |
| 14 | i16 | Inverter torque command 50 ms earlier | same |
| 16 | i16 | Commanded torque, 0.1 Nm | 0x473 B0–1, BE |
| 18 | i16 | Motor speed raw | 0x181 with B0 = 0x30: B1–2, LE |
| 20 | i16 | DC bus voltage raw | 0x181 with B0 = 0xEB (or 0x66): B1–2, LE |
| 22 | u16 | Pack current raw | 0x301 B0–1, BE |
| 24 | u16 | Pack instant voltage raw | 0x301 B2–3, BE |
| 26 | u8 | Pack SOC | 0x301 B4 |
| 27 | u8 | Max cell temp °C | 0x1839F380 (extended) B2 |
| 28 | u8 | BMS fault number | 0x588 B0 |
| 29 | u8 | Reserved (0) | — |
| 30 | i16 | Motor temp raw | 0x181 with B0 = 0x49: B1–2, LE |
| 32 | i16 | IGBT temp raw | 0x181 with B0 = 0x4A: B1–2, LE |
| 34 | i16 | Actual current raw (I_IST) | 0x181 with B0 = 0x20: B1–2, LE |
| 36 | u16 | Bamocar status KERN_STATUS | 0x181 with B0 = 0x40: B1–2, LE |
| 38 | u32 | BMS fault flags | 0x303 B0–3, BE |
| 42 | u16 | Pack open-circuit voltage raw | 0x301 B5–6, BE |
| 44 | u16 | Pack resistance raw | 0x302 B2–3, BE |
| 46 | u8 | BMS high temp | 0x302 B4 |
| 47 | u8 | BMS low temp | 0x302 B5 |
| 48 | u16 | SOC (16-bit) | 0x302 B0–1, BE |
| 50 | u16 | CAN frames received in the last 100 ms | STM32 |
| 52 | u8 | Last command sequence number acknowledged | STM32 (0 if none) |
| 53 | u8 | Its status (3.8); 0xFF = no command yet | STM32 |

Bytes 52–53 put an acknowledgement in every batch, so the PC sees it within 200 ms.

"50 ms earlier" values: sample brake1, brake2 and the torque command every 50 ms. FAST is
built every 100 ms from the two latest samples.

A 0x181 frame from the Bamocar is: B0 = register ID, B1..B2 (16-bit) or B1..B4 (32-bit)
value LE, plus one filler byte. DLC 4 = 16-bit, DLC 6 = 32-bit.

### 3.3 FAST `valid` and `status` bits

`valid` bit = 1 when that source was seen within its timeout:

| Bit | Source | Timeout |
|---|---|---|
| 0 | 0x301 | 300 ms |
| 1 | 0x302 | 300 ms |
| 2 | 0x303 | 300 ms |
| 3 | 0x1839F380 | 300 ms |
| 4 | 0x181 reg 0x30 (speed) | 300 ms |
| 5 | 0x181 reg 0x49 or 0x4A (temps) | 300 ms |
| 6 | 0x181 reg 0x20 (current) | 2 s |
| 7 | 0x181 reg 0x40 (KERN_STATUS) | 2 s |
| 8 | 0x181 reg 0xEB or 0x66 (DC bus) | 2 s |
| 9 | 0x364 | 200 ms |
| 10 | 0x365 | 200 ms |
| 11 | 0x473 (only sent while driving) | 500 ms |
| 12 | 0x201 reg 0x90 (torque command) | 500 ms |
| 13 | 0x471 (GEVCU, sent every 10 ms) | 100 ms |
| 14 | 0x588 | 2 s |
| 15 | 0x18EEFF80 (extended, BMS thermistor module heartbeat) | 1 s |

`status` bits:

| Bit | Meaning |
|---|---|
| 0 | Red light: last 0x470 B0 ≠ 0 |
| 1 | BSPD OK: last 0x471 B0 == 1 |
| 2 | 0x468 seen in the last 100 ms (GEVCU sends it only while its BMS fault is set) |
| 3 | 0x469 seen in the last 100 ms (GEVCU sends it only while its IMD fault is latched) |
| 4 | R2D: last 0x475 B0 |
| 5 | Inverter enabled: the last 0x201 frame with B0 = 0x51 had bit 2 of its value clear (`51 00 00` = enable, `51 04 00` = disable) |
| 6 | 0x777 (dash button) seen since the previous FAST |
| 7 | Diagnostics session active |
| 8 | CAN error-passive or bus-off |
| 9–15 | Reserved (0) |

### 3.4 THERM payload (type 2, 1 Hz)

What frames 0x1839F381–385 contain isn't confirmed (individual thermistors, or per-module
summaries). So THERM carries them **raw**, and the PC decodes them later.

| Off | Field |
|---|---|
| 0–47 | Last 8 data bytes of each of 0x1839F380, 381, 382, 383, 384, 385 (6 × 8 B, in that order) |
| 48 | Bitmap: bit k = frame 0x1839F380 + k received in the last 2 s; bit 6 = 0x476 |
| 49–50 | 0x476 B0–1 as received: motor temperature relayed by the VCU, integer °C (added 2026-11-06; the PC decodes it big-endian, signed) |
| 51–53 | Reserved |

### 3.5 DIAG payload (type 3, 1 Hz, also used as filler)

| Off | Type | Field |
|---|---|---|
| 0 | u16 | Firmware version |
| 2 | u8 | Build flag `TELEM_BAMO_DIAG` (0/1/2) |
| 3 | u8 | Bit 0: `DIAG_ALLOW_WRITES`; bit 1: `DIAG_ANY_ID` |
| 4 | u32 | CAN frames received, total |
| 8 | u32 | CAN frames whose ID isn't used by anything above |
| 12 | u8 | CAN transmit error counter |
| 13 | u8 | CAN receive error counter |
| 14 | u8 | Bus-off count |
| 15 | u8 | CAN receive overflow count |
| 16 | u16 | `channel Error` lines from the car module |
| 18 | u16 | `Power on` lines from the car module |
| 20 | u16 | Command candidates with bad CRC or bad key |
| 22 | u16 | Commands executed |
| 24 | i16 | Nmax, reg 0xC8 (speed scaling) |
| 26 | i16 | Ramp, reg 0x25 |
| 28 | i16 | I_MAX, reg 0x4D |
| 30 | i16 | I_MAX_PK, reg 0xC4 |
| 32 | i16 | I_CON_EFF, reg 0xC5 |
| 34 | i16 | I device, reg 0xC6 (current scaling) |
| 36 | i16 | I 200%, reg 0xD9 (current scaling) |
| 38 | u16 | Bitmap: which of offsets 24–36 have been received (bit 0 = 0xC8 … bit 6 = 0xD9) |
| 40 | u16 | 0x481 B0–1 raw (sender unknown) |
| 42 | u16 | 0x666 B0–1 raw (sender unknown) |
| 44 | u16 | 0x321 B0–1 raw (sender unknown) |
| 46 | u8 | Bitmap: 0x481 / 0x666 / 0x321 seen in the last 2 s |
| 47 | u8 | Events dropped (saturating) |
| 48 | u16 | Diagnostic capture frames dropped (saturating) |
| 50–53 | | Reserved |

The register values at 24–36 arrive on 0x181 whenever anything requests them (the GEVCU
already requests 0xC8, 0x25, 0x4D, 0xC4, 0xC5). Store the latest value of each.

### 3.6 EVENT payload (type 4)

| Off | Field |
|---|---|
| 0 | Count (0–10) |
| 1–50 | Up to 10 records of 5 B: `u16 age_ms` (packet uptime − event time, saturating at 65535), `u8 code`, `u16 value` |
| 51–53 | Reserved |

Keep events in a ring buffer of 32. If it overflows, drop the oldest and count it (DIAG
offset 47, and code 19).

| Code | Event | Value |
|---|---|---|
| 1 | 0x777 received (dash button) | B0–1 of the frame |
| 2 / 3 | R2D on / off (edge of 0x475 B0) | — |
| 4 / 5 | BSPD trip / clear (0x471 B0 1→0 / 0→1) | — |
| 6 / 7 | 0x468 starts / stops arriving | — |
| 8 / 9 | 0x469 starts / stops arriving | — |
| 10 | BMS fault number changed (0x588) | New number |
| 11 | Inverter enable/disable command seen (0x201, B0 = 0x51) | The 16-bit value |
| 12 | BMS clear sent (0x7E3) | B0–1 |
| 13 | Bamocar error clear sent (0x201, B0 = 0x8E) | — |
| 14 / 15 | Source timed out / came back | `valid` bit number |
| 16 / 17 | CAN bus-off / error-passive | — |
| 18 | Car LoRa module printed `Power on` | — |
| 19 | Events were dropped | Count |

"Starts / stops arriving" for 0x468/0x469: start = first frame after 100 ms without one;
stop = 100 ms without one.

### 3.7 Command packet (PC → car): 24 B

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Sync `A5 5A` |
| 2 | 1 | `0x18` (version 1, type 8) |
| 3 | 1 | Command sequence number (u8; PC increments it per new command) |
| 4 | 2 | Car key `5A C3` (compile-time constant; not security, it only stops stray packets from other teams' radios being acted on) |
| 6 | 1 | Command code |
| 7 | 15 | Arguments (unused = 0) |
| 22 | 2 | CRC over bytes 0–21 |

| Code | Command | Arguments (offset within args) |
|---|---|---|
| 0x01 | PING (starts or keeps a session) | — |
| 0x02 | BAMO_READ | 0: register; 1: period (0 = once, 20–254 = every N ms, 0xFF = stop) |
| 0x03 | BAMO_WRITE | 0: register; 1: size (2 or 4); 2–5: value u32 |
| 0x04 | SEND_FRAME | 0–3: ID u32 (bit 31 = extended); 4: DLC; 5–12: data |
| 0x05 | CAPTURE_SET | 0: slot 0–3; 1–4: ID u32 (bit 31 = extended); 5–8: mask u32; 9: max rate, frames/s |
| 0x06 | CAPTURE_CLEAR | 0: slot, 0xFF = all |
| 0x07 | STOP_ALL | — |

BAMO_READ sends `3D <reg> <period>` on 0x201 (DLC 3). BAMO_WRITE sends `<reg> <value LE>`
on 0x201 with DLC 3 (2-byte) or 5 (4-byte).

**Repeats:** the PC resends a command until acknowledged. If a command arrives with the
same sequence number as the last one executed, **don't run it again**: acknowledge it again
with the stored status.

### 3.8 DIAG_RESP payload (type 5) and status codes

| Off | Field |
|---|---|
| 0 | Number of acknowledgements (0–3) |
| 1–6 | 3 × `{u8 seq, u8 status}` |
| 7 | Number of frames (0–3) |
| 8–52 | 3 × 15 B frames: `u16 age_ms`, `u32 id` (bit 31 = extended), `u8 dlc`, `u8 data[8]` |
| 53 | Capture frames dropped since the last DIAG_RESP (saturating) |

Status codes: 0 OK · 2 REJ_DISABLED (build doesn't allow it) · 3 REJ_ID (CAN ID not
allowed) · 4 REJ_REGID (register blocked) · 5 REJ_INV_ENABLED (inverter may be live) ·
6 REJ_RATE · 7 REJ_LIMIT (too many repeating reads) · 8 BAD_ARGS · 10 TX_FAIL (CAN send
failed, e.g. bus-off).

Bad CRC or wrong key: ignore the packet entirely (no acknowledgement) and count it.

---

## 4. Shared schema (single source of truth)

Write every packet layout from section 3 into `telemetry/schema/telemetry_schema.yaml`:
for each packet type and field, its offset, type, name, CAN source, CAN byte order, unit,
scale and offset for display, plus the bit tables, event codes and status codes.

Write `telemetry/schema/gen_schema.py`. It generates `telemetry_schema.h` for the
firmware:
- packed structs for every payload, with `_Static_assert(sizeof(...) == 54)` (24 for the
  command),
- `#define`s for types, event codes, status codes, bits and offsets.

The firmware includes the generated header. The PC tools read the YAML directly at runtime.
Commit the generated header too, and add a check (`gen_schema.py --check`) that fails if it
is out of date.

**Display scaling (YAML `scale`), known so far:**
- Motor speed rpm = raw / 32767 × Nmax (DIAG offset 24).
- Motor temp °C = (raw − 9369) / 55.21 (from the GEVCU code).
- DC bus V = raw × factor for the Bamocar model (manual p.20: 0.031663506 for 700-100 and
  700-160; 0.0181423 for 400-400 and 700-250). **Model unknown: leave a TODO and show raw.**
- Torque command % = raw / 32767 × 100.
- Commanded torque Nm = raw / 10.
- BMS values: leave scale 1 with a TODO (decided later).

---

## 5. Car firmware

### 5.1 Structure
Keep hardware code and logic apart, so the logic can be tested on a PC (section 8):
- `telem_core.c/.h`: signal table, CAN frame decoding, timeouts, event detection, packet
  building, CRC, batch scheduling decisions. **No HAL includes.**
- `diag_core.c/.h`: command parsing, validation, session state, repeating-read and capture
  bookkeeping. **No HAL includes.** It returns CAN frames to send; it doesn't send them.
- `main.c` / `board.c`: HAL glue: CAN init and interrupt, UART DMA, timers, watchdog.

### 5.2 Receiving CAN
- One filter bank accepting everything (mask 0), FIFO0, interrupt.
- The interrupt copies `{id, ext flag, dlc, data[8], time ms}` into a 32-entry ring
  buffer. The main loop decodes them. Count overflows.
- Decode only what section 3 lists. Count everything else as "unused".
- Also watch the GEVCU's own 0x201 frames (B0 = 0x90 torque, 0x51 enable/disable, 0x8E
  error clear) and 0x7E3.

### 5.3 Sending
- Use `HAL_GetTick()` deadlines: next write at `t0 + 200 ms × k`, not "200 ms after the
  last one".
- Each write: build the 3 packets into one 195 B static buffer (header first), start one
  DMA transfer. Refuse to start a write if the previous DMA is still running, if it would
  start less than 120 ms after the previous write, or if it is over 250 B.
- At power-up, wait 1 s before the first write (the module prints `Power on` ~0.5 s after
  boot).
- Add the independent watchdog (IWDG), about 500 ms.

### 5.4 Receiving from the module
- Stream parser: find `A5 5A`, take 24 bytes, check CRC and key. On failure move ahead
  one byte.
- Separately match `channel Error` and `Power on` in the byte stream and count them.
- In the build with flag 0, still count those lines, but ignore commands.

### 5.5 Build flags
| Flag | Values |
|---|---|
| `TELEM_BAMO_DIAG` | **0** (default for the car): telemetry only, CAN controller in **silent mode** (hardware can't transmit), commands ignored. **1**: telemetry + diagnostics, CAN in normal mode. **2**: diagnostics first (3.1). |
| `DIAG_ALLOW_WRITES` | **1** (Bamocar parameter writes allowed, subject to 5.6) |
| `DIAG_ANY_ID` | 0 (SEND_FRAME only to 0x201). 1 allows any ID; bench use only |
| `CMD_KEY` | `0x5AC3` |

### 5.6 Diagnostics safety rules (flags 1 and 2)
These are not optional. Put them in `diag_core.c` and test them (section 8).

1. **Inverter may be live** if any of these is true:
   - the last 0x201 frame with B0 = 0x51 enabled it (bit 2 clear),
   - 0x475 B0 = 1 seen in the last 10 s,
   - 0x473 seen in the last 2 s,
   - KERN_STATUS (0x40) bit 0 set, seen in the last 2 s.

   While live, allow only PING, CAPTURE_SET/CLEAR, STOP_ALL, and BAMO_READ with
   period 0. Everything else: REJ_INV_ENABLED.
2. **Blocked registers** for BAMO_WRITE, and for SEND_FRAME on 0x201 (data byte 0 is the
   register): `0x90` torque, `0x31` speed, `0x6E` position, `0x51` mode/enable,
   `0x68` CAN receive ID, `0x84` EEPROM write, `0xDC` configuration (bit 15 switches the
   inverter to CANopen). Return REJ_REGID. Parameter writes therefore only change RAM
   and are lost when the inverter powers off. That's intended.
3. **Reads of the GEVCU's repeating registers** (0x30, 0x49, 0x4A) are blocked
   (REJ_REGID). They already arrive every 100 ms, and a one-shot or stop request could
   cancel the GEVCU's repeating setting.
4. A SEND_FRAME on 0x201 with byte 0 = 0x3D follows the BAMO_READ rules.
5. **Repeating reads:** at most 3 started by diagnostics at once (the Bamocar supports 8;
   the GEVCU uses up to 5), period at least 20 ms. Otherwise REJ_LIMIT / BAD_ARGS.
6. **Rate limit:** diagnostics send at most 20 CAN frames per second in total
   (REJ_RATE).
7. **Session:** starts with any valid command, ends with STOP_ALL or **10 s without a
   valid command**. Ending a session sends `3D <reg> FF` for every repeating read it
   started, and clears all captures.
8. **Captures:** up to 4 `{id, mask, extended, max rate}` slots. Matching received frames
   go into a 16-entry ring for DIAG_RESP; count drops. Default when a session starts:
   none (the PC sets slot 0 to 0x181 itself).

---

## 6. Build outputs

### 6.1 Firmware
Two build configurations in the CubeIDE project: `Telemetry` (flag 0) and `Diag`
(flag 1). Report flash and RAM use of both.

### 6.2 GEVCU changes (separate repo, not part of this task)
Two telemetry fields only fill once the GEVCU (repo `gevcu-2024-2025`) is changed:
KERN_STATUS (0x40) and DC bus voltage (0xEB). The GEVCU currently sends a malformed DC bus
request (`EB 3D …` instead of `3D EB …`) and never requests 0x40. Someone will change that
separately. Until then those `valid` bits stay 0. That's expected; don't work around it.

---

## 7. PC tools (Python 3.10+, pyserial, PyYAML)

Folder `telemetry/pc/`.

### 7.1 `lora_link.py`: owns the serial port
- One reader thread: read with `time.perf_counter()` timestamps.
- Stream parser for 64 B packets (sync, length, CRC; on failure move ahead one byte).
  Skip `Power on` / `channel Error` text, and report `Power on` as an event.
- **Batch arrival:** after at least 64 B arrived, 10 ms with no more bytes. The arrival
  time is the timestamp of the last read.
- **Command sender:** at most one command waiting for acknowledgement.
  - Send only at **arrival + 110 ms** (configurable; busy-wait the last ~20 ms, since
    `time.sleep` is coarse on Windows). Header `0A 02 03`, one 24 B command per write.
  - Never start a write less than 150 ms after the previous PC write.
  - Acknowledged when FAST bytes 52–53 or a DIAG_RESP acknowledgement carries its
    sequence number. No acknowledgement within 5 batches (1 s): resend at the next
    arrival + 110 ms. Give up after 10 tries and report failure.
  - While a session is open and nothing else is waiting, send PING every 3 s.

### 7.2 `decoder.py`
Loads the YAML and turns a packet into a dict of named, scaled values. Bamocar frames in
DIAG_RESP: register = B0, value LE, 16-bit for DLC 4, 32-bit for DLC 6.

### 7.3 Logging
- Raw log of every good packet: `logs/raw_<date>_<time>.bin`, records
  `[f64 PC time][64 B packet]`. This is the master record; everything else can be
  rebuilt from it.
- CSV per packet type (`fast.csv`, `therm.csv`, `diag.csv`, `events.csv`,
  `diag_resp.csv`) plus `commands.csv` (every command sent, each attempt, outcome).
- `replay.py <raw.bin>` rebuilds the CSVs from a raw log.

### 7.4 PC module lockup
- **Detect:** 3 resends in a row without acknowledgement **and** at least one missing
  packet (sequence gap) during those resends.
- **Then:** stop sending commands, show "PC radio restarting, commands paused".
- **Resume:** when `Power on` appears on the port, or after 90 s.
- Telemetry keeps being received and logged throughout (expect ~20% loss while locked).
- Don't send AT commands to recover. It was never tested during a lockup.

### 7.5 `telemetry_cli.py`
`py telemetry_cli.py --port COM9 [--delay-ms 110]`

- **Live status line**, twice a second: key values (speed rpm, torque command, pack
  voltage/current/SOC, max cell temp, brakes, status bits), packets/s, loss %, link state
  (OK / PC radio restarting).
- **Diagnostics prompt:**
  - `ping`
  - `read <reg> [--every <ms>]`, `stop <reg>`
  - `write <reg> <value> [--32]`
  - `raw <id> <bytes…>`
  - `capture <id> [<mask>] [--slot N] [--rate N]`, `uncapture [<slot>]`
  - `stopall`, `status`, `quit`

  Each command shows its acknowledgement status in words. Received Bamocar values are
  shown by register name (include a name table for at least the registers in this
  document).
- **Warn before** `read 0x8F` (error register): the GEVCU automatically sends an error
  clear when it sees that reply with byte 1 = 0x20.

---

## 8. Tests you must run (no radio hardware needed)

1. **Host tests of the firmware logic:** compile `telem_core.c` and `diag_core.c` with the
   PC's gcc against a small test runner (`telemetry/tests/host/`). Cover at least:
   - CRC test vector (`"123456789"` → `0x29B1`).
   - Every CAN source decodes into the right FAST field with the right byte order (use
     the exact frames in section 3, e.g. 0x181 `30 FF 7F 00` → speed 32767).
   - `valid` bits clear after their timeouts; events fire on the edges in 3.6, and
     the event ring drops the oldest on overflow.
   - Batch builder: always 3 packets, packet 1 is the FAST from 100 ms earlier, slot
     rotation follows 3.1, every write is 195 B with header `0A 01 03`.
   - Command validator: every rule in 5.6, including blocked registers via both
     BAMO_WRITE and SEND_FRAME, the live-inverter rules, the repeating-read limit, the rate
     limit, session timeout sending `3D <reg> FF`, and same-sequence repeats not running
     twice.
   - Bad CRC / bad key commands are ignored and counted.
2. **Python tests** (`pytest`): CRC; encode → decode round trip for every packet type
   from the YAML; the stream parser on garbage, merged packets, a cut packet and
   `Power on` text; the command sender's acknowledgement, resend, give-up and lockup
   states using a fake clock and fake port.
3. **Cross-check:** a C program in the host tests writes one packet of each type to a
   file; a Python test decodes them and checks every field. This proves both sides agree.
4. `gen_schema.py --check` passes.

---

## 9. Bench acceptance (needs the board and both modules; list it, the team runs it)

Write `telemetry/BENCH_TEST.md` with these steps and pass criteria:
1. `Telemetry` build, CAN connected, 10 min, no commands: 15 packets/s, **0 lost**, car
   `channel Error` count 0.
2. `Diag` build, a session open with a `read` every 2 s for 10 min: all commands
   acknowledged except during a PC-module lockup; telemetry loss 0 outside lockups.
3. Safety: with the inverter powered but not enabled, `write 0x90 …` and `write 0x51 …`
   are rejected (REJ_REGID); with R2D active, `write 0x35 …` is rejected
   (REJ_INV_ENABLED).

---

## 10. Deliverables

- Firmware (both builds compile; sizes reported).
- `telemetry_schema.yaml`, `gen_schema.py`, generated header.
- PC tools in `telemetry/pc/`, with `requirements.txt` and a short `README.md` (how to run,
  how to read the CSVs).
- Host tests and pytest suite, all passing; paste the output in your final message.
- `BENCH_TEST.md`.
- Commit on `lora-telemetry` and push.

Report anything you couldn't do or had to change from this document, and why. If a rule
here seems wrong, ask before changing it.
