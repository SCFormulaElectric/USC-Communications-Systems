# Build task: Arduino CAN-bus simulator for the LoRa telemetry board

Write an Arduino sketch that plays the car's CAN bus on the bench. The LoRa telemetry board
(`LoRa_board`, firmware in `telemetry/`) can then be tested end to end without the car: telemetry
values, `valid` bits, events, and the Bamocar diagnostics including the safety rules.

Hardware: **Arduino Uno + Seeed Studio CAN-BUS Shield V2.0** (MCP2515 controller + MCP2551
transceiver, 16 MHz crystal).

Read first: `telemetry/BUILD_INSTRUCTIONS.md` section 3 (which frames the board decodes and how),
and `telemetry/IMPLEMENTATION_REPORT.md` section 4 (how the firmware interprets edge cases). The
frame layouts below come from those documents and `telemetry/schema/telemetry_schema.yaml`. If
they disagree, the YAML and the firmware (`LoRa_board/Core/Src/telem_core.c`) win; tell the user.

Work on branch `lora-telemetry`. Put everything in `2025-2026/Lora-Code/telemetry/can_sim/`.

---

## 1. Hardware facts (verify against the board and the Seeed wiki)

| Item | Value |
|---|---|
| Library | Seeed `Seeed_Arduino_CAN` (Library Manager: "CAN_BUS_Shield" / "Seeed Arduino CAN"). Recent versions: `#include "mcp2515_can.h"`, `mcp2515_can CAN(SPI_CS_PIN);`. Older: `mcp_can.h` / `MCP_CAN`. Check which API the installed version has. |
| SPI chip select | **D9** on V2.0 (D10 on V1.0 and older): check the silkscreen |
| Interrupt | D2 (MCP2515 INT) |
| Crystal | 16 MHz: `CAN.begin(CAN_500KBPS, MCP_16MHz)` (or `CAN.begin(CAN_500KBPS)` on older versions) |
| Bit rate | **500 kbit/s** (the car bus; the LoRa board is 500 kbit/s) |
| Wiring | Shield CAN_H / CAN_L (screw terminal, or check the DB9 pinout on the wiki) to the LoRa board's CAN_H / CAN_L, plus **GND** |
| Termination | 120 Ω at each end of the bus, 60 Ω total between CAN_H and CAN_L with power off. The shield has a 120 Ω resistor (check whether it's enabled). |

**ACK problem (must be in the README):**
- In the `Telemetry` build (flag 0) the LoRa board's CAN controller is in **silent mode**: it
  never acknowledges frames.
- With only the Arduino and the LoRa board on the bus, nothing ACKs the Arduino's frames. The
  MCP2515 then retransmits the same frame forever, the board receives nothing valid, and the
  Arduino goes error-passive.
- Testing the Telemetry build therefore needs a **third node in normal mode** on the bus. A
  USB-CAN adapter (e.g. the one `2026-2027/can_viewer` uses) or a second MCP2515 is enough.
- The `Diag` build (flag 1) is in normal mode and ACKs by itself.
- The sketch should detect this: if TX error counter > 127 or frames stop leaving, print
  "no ACK on the bus: add a node in normal mode, or use the Diag build".

## 2. What the sketch must do

### 2.1 Play every node the board listens to

Sends the frames below on their own schedules, using `millis()` deadlines (`next += period`, not
"period after the last send"). All IDs are 11-bit unless marked ext (29-bit).

- **Byte order matters.** Values marked BE are big-endian on the bus; LE are little-endian.
  Bamocar frames on 0x181 are `B0 = register, B1..B2 = value LE, B3 = 0`, DLC 4.
- **Rates marked "assumed" aren't known for the real car.** Make them easy to change (a table at
  the top of the sketch). The `valid` timeouts in the brief are the upper limits.

| ID | Node | Contents | Period | Notes |
|---|---|---|---|---|
| 0x471 | GEVCU | B0 = BSPD OK (1) / tripped (0) | **10 ms** | brief: every 10 ms; valid timeout 100 ms |
| 0x470 | GEVCU | B0 = red light (0 off) | 100 ms (assumed) | |
| 0x475 | GEVCU | B0 = R2D (0/1) | 100 ms (assumed) | |
| 0x473 | GEVCU | B0–1 = commanded torque, BE i16, 0.1 Nm | 20 ms (assumed), **only while driving** | |
| 0x201 | GEVCU → Bamocar | `90 <torque LE i16>` torque command, 0x7FFF = 100% | 20 ms (assumed) | 0 when not driving |
| 0x201 | GEVCU → Bamocar | `51 00 00` enable / `51 04 00` disable | on change + every 500 ms (assumed) | |
| 0x181 | Bamocar | reg 0x30 speed, 0x49 motor temp, 0x4A IGBT temp (LE i16) | 100 ms each | the GEVCU's repeating reads |
| 0x181 | Bamocar | reg 0x20 actual current | 100 ms (assumed) | |
| 0x181 | Bamocar | reg 0xC8 Nmax, 0x25, 0x4D, 0xC4, 0xC5 | every 5 s (assumed) | replies to the GEVCU's requests; Nmax gives the rpm scaling |
| 0x181 | Bamocar | reg 0x40 KERN_STATUS, 0xEB DC bus | 500 ms, **off by default** | the real GEVCU doesn't request these yet (brief 6.2); a serial command turns them on |
| 0x301 | BMS | B0–1 current BE, B2–3 voltage BE, B4 SOC, B5–6 OCV BE | 100 ms (assumed) | |
| 0x302 | BMS | B0–1 SOC16 BE, B2–3 resistance BE, B4 high temp, B5 low temp | 100 ms (assumed) | |
| 0x303 | BMS | B0–3 fault flags BE u32 | 100 ms (assumed) | |
| 0x588 | BMS | B0 = fault number | 500 ms (assumed) | |
| 0x476 | VCU | B0–1 = motor temperature, integer °C, **BE i16** (byte order unconfirmed: make it a switch) | 500 ms (assumed) | THERM bytes 49–50; `seen` bit 6 |
| 0x1839F380 ext | thermistor module | B2 = max cell temp °C (8 bytes) | 100 ms (assumed) | |
| 0x1839F381–385 ext | thermistor module | 8 bytes each, any recognisable pattern | 500 ms (assumed) | the board forwards them raw |
| 0x18EEFF80 ext | thermistor module heartbeat | any | 200 ms (assumed) | valid timeout 1 s |
| 0x364 / 0x365 | brakes | B0–1 BE u16 | 20 ms (assumed) | valid timeout 200 ms |
| 0x481, 0x666, 0x321 | unknown senders | B0–1 any (e.g. `12 34`) | 500 ms (assumed) | |
| 0x468 / 0x469 | GEVCU | BMS fault / IMD fault messages | every 50 ms **only while that fault is active** | status bit = seen in the last 100 ms |
| 0x777 | dash | B0–1 button value | on a serial command | event 1 |
| 0x7E3 | BMS clear | B0–1 | on a serial command | event 12 |
| 0x201 | GEVCU | `8E 00 00` Bamocar error clear | on a serial command | event 13 |
| 0x123 | noise | any | 0/s by default; set 100–1000/s for a load test | counted as "unused" in DIAG |

Default load is about 400 frames/s, roughly 10% of a 500 kbit/s bus. Print the actual frames/s
once a second so the load is known.

### 2.2 Scenarios (serial monitor, 115200 baud, single-letter commands)

- `i` **idle** (default at boot): car on, not ready to drive.
  - R2D 0, inverter disabled (`51 04 00`), torque 0, speed 0.
  - BSPD OK, no faults.
- `d` **drive**:
  - R2D 1, inverter enabled (`51 00 00`).
  - Torque command ramps 0 → 50% over 5 s and back; speed follows a sine 0 → 16384 raw
    (= Nmax / 2 = 3000 rpm with Nmax 6000).
  - 0x473 is sent; brakes vary; motor temp rises slowly.
- `b` toggle a **BSPD trip** (0x471 B0 1 → 0, and back).
- `f` / `m` toggle the **BMS-fault (0x468) / IMD-fault (0x469)** message bursts.
- `n` change the **BMS fault number** (0x588 B0: 0 → 3 → 0).
- `p` **dash button** (one 0x777 frame); `c` **BMS clear** (0x7E3); `e` **Bamocar error clear**
  (0x201 `8E`).
- `t` **timeout test**: stop 0x301 for 2 s, then resume. The board should time out `valid` bit 0
  and report it coming back.
- `g` toggle the 0x40 / 0xEB replies (simulates the GEVCU change).
- `s` cycle the noise rate: 0 → 200 → 1000 frames/s → 0.
- `?` print the state, rates, frames/s, TX/RX error counters, and the Bamocar register table.

### 2.3 Emulate the Bamocar for diagnostics (Diag build)

The LoRa board's diagnostics send 0x201 frames. The sketch receives them (interrupt on D2, or
poll `checkReceive()`) and acts like the Bamocar:

- `3D <reg> <period>`: a read. Period 0: reply once on 0x181 (`reg lo hi 00`, DLC 4). Period
  20–254: reply every `period` ms until stopped (at most 8 repeating reads at once). Period 0xFF:
  stop.
- `<reg> <lo> <hi>` (DLC 3) or `<reg> b0 b1 b2 b3` (DLC 5): a parameter write. Store it in a RAM
  register table so a later read returns the new value. The real Bamocar doesn't acknowledge
  writes; don't invent a reply.
- Register table with at least: 0xC8 Nmax 6000, 0x25, 0x4D, 0xC4, 0xC5, 0xC6, 0xD9, 0x35 (accel
  ramp), 0x8F errors = 0. Unknown registers read as 0.
- **Print every frame received from the LoRa board** on the serial monitor, with a timestamp,
  e.g. `RX 0x201 [3] 3D C8 00  read N_MAX once`.
  - This is the safety check for bench test 3: a blocked write (0x90, 0x31, 0x6E, 0x51, 0x68,
    0x84, 0xDC) must **never** appear.
  - If one does, print `!!! BLOCKED REGISTER ON THE BUS` in a way that's hard to miss.

Don't let the sketch's own 0x201 frames (it's also playing the GEVCU) count as diagnostics
traffic. The MCP2515 doesn't receive its own frames, so this happens naturally; just don't loop
them back in software.

## 3. Values: make them easy to check on the PC

Use fixed, recognisable values in `idle`, so `telemetry_cli.py` and `fast.csv` can be checked
by eye. Suggested values (decoded as `telem_core.c` does):

| Frame (idle) | Bytes | Telemetry shows |
|---|---|---|
| 0x301 | `00 64 0D 80 55 0D 90 00` | pack current 100, voltage 3456, SOC 85, OCV 3472 |
| 0x302 | `21 34 00 2A 2D 19` | SOC16 8500, resistance 42, BMS high 45, low 25 |
| 0x303 | `00 00 00 00` | fault flags 0 |
| 0x1839F380 | `01 02 20 04 05 06 07 08` | max cell temp 32 °C |
| 0x588 | `00` | BMS fault number 0 |
| 0x476 | `00 28` | THERM motor temp 40 °C; status line `motor 40C` |
| 0x364 / 0x365 | `01 00` / `01 02` | brake1 256, brake2 258 |
| 0x181 0x49 | `49 39 2D 00` (raw 11577) | motor temp ≈ 40.0 °C ((raw − 9369) / 55.21) |
| 0x181 0x4A | `4A E0 2E 00` | IGBT raw 12000 |
| 0x181 0xC8 | `C8 70 17 00` | Nmax 6000 (DIAG); speed then shows in rpm |
| 0x471 / 0x470 / 0x475 | `01` / `00` / `00` | status: `bspd_ok` only |

In `drive`, the status line should also show `r2d inv_enabled`, rpm up to ~3000, and torque up
to 50%.

## 4. Deliverables

1. `telemetry/can_sim/can_sim.ino` (one sketch; split into tabs only if it helps).
   - Must fit an **Uno: 32 KB flash, 2 KB RAM**.
   - No `String`, no dynamic memory. Use `F()` for printed text and `PROGMEM` for tables.
   - Report flash/RAM from the compile.
2. `telemetry/can_sim/README.md` covering:
   - wiring, termination and **the ACK problem** (section 1)
   - installing the library and the CS pin
   - the serial commands
   - a **test checklist**: for each scenario command, what to look for on the PC (status line,
     `fast.csv` columns, `events.csv` event names, `diag_resp.csv`, and the sketch's own RX log)
3. Compile check: use `arduino-cli compile --fqbn arduino:avr:uno` if `arduino-cli` is available
   (or offer to install it). If you can't compile, say so; don't claim it builds.
4. Commit on `lora-telemetry` and push.

Don't change the LoRa board firmware or the telemetry schema. If something in them looks wrong
while you work, report it and ask first.

## 5. Checklist the README should include (minimum)

| Do | Expect on the PC (`py telemetry_cli.py --port <PC module>`) |
|---|---|
| power up in `idle` | 15.0 pkt/s, loss 0%; status `bspd_ok`. `valid` has every source except 0x473 (bit 11) and, until `g`, 0x40 / 0xEB (bits 7, 8) |
| `d` | `r2d inv_enabled`; rpm and torque rising; events R2D_ON, INV_ENABLE_CMD (value 0), SOURCE_BACK for bit 11 |
| `i` | events R2D_OFF, INV_ENABLE_CMD (value 4); bit 11 times out after 500 ms (SOURCE_TIMEOUT 11) |
| `b` twice | BSPD_TRIP then BSPD_CLEAR; `bspd_ok` goes away then returns |
| `f` on/off | BMS_FAULT_MSG_START / STOP; status `bms_fault_msg` while on |
| `n` | BMS_FAULT_CHANGED with value 3, then 0 |
| `p`, `c`, `e` | DASH_BUTTON, BMS_CLEAR_SENT, BAMO_ERR_CLEAR_SENT |
| `t` | SOURCE_TIMEOUT 0, then SOURCE_BACK 0 about 2 s later |
| `s` (1000/s) | DIAG `can_rx_unused` climbs ~1000/s; `can_rx_overflow` stays 0; FAST `can_frames_100ms` ≈ 140 |
| Diag build: `read N_MAX` | the sketch logs `RX 0x201 3D C8 00`; the PC prints `N_MAX (0xC8) = 6000`. The PC may miss the reply: IMPLEMENTATION_REPORT section 5 |
| Diag build, in `idle` for at least 10 s (the R2D live window): `write 0x35 500`, then `read 0x35` | the sketch logs the write and its table shows 500 |
| Diag build: `write 0x90 0` | PC: `rejected: register blocked`; the sketch logs **nothing** for it |
| Diag build, `d` (driving): `write 0x35 1` | PC: `rejected: inverter may be live`; the sketch logs nothing |
