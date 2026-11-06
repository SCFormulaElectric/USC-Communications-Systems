# LoRa telemetry and Bamocar diagnostics: implementation report

Implements `BUILD_INSTRUCTIONS.md` on branch `lora-telemetry`, 2026-11-05.
- **Firmware:** both builds compile and fit.
- **Host tests:** all pass, in C and Python.
- **Not done:** the bench acceptance test (needs the hardware; steps in `BENCH_TEST.md`).

**Needs a decision:** in the Diag build, a Bamocar read reply gets back to the PC only about
1 time in 5 (section 5). A one-line fix gives 100% in simulation, but it changes a rule in the
brief, so it isn't applied.

## 1. What was built

| Part | Files |
|---|---|
| Schema (single source of truth) | `telemetry/schema/telemetry_schema.yaml` |
| Generator | `telemetry/schema/gen_schema.py` (outputs below; `--check` fails if stale) |
| Generated header (committed) | `LoRa_board/Core/Inc/telemetry_schema.h`: packed structs, 121 `_Static_assert`s on every size and offset, `#define`s for types, codes, bits, timeouts and blocked registers |
| Firmware logic, no HAL | `LoRa_board/Core/Src/telem_core.c` (CAN decoding, valid/status bits, events, packet and batch building, CRC, LoRa receive parser) and `diag_core.c` (command validation, safety rules, session, repeating reads, captures, DIAG_RESP) |
| Firmware glue | `main.c` (CubeMX layout, loop), `board.c` (CAN ring, USART2 + DMA, IWDG), `telem_config.h` (build flags) |
| PC tools | `telemetry/pc/`: `lora_link.py`, `decoder.py`, `logger.py`, `replay.py`, `telemetry_cli.py`, `read_loop.py`, `README.md`, `requirements.txt` |
| Tests | `telemetry/tests/host/` (C runner, cross-check writer, generated filler), `telemetry/tests/*.py` (pytest) |
| Command-line build + size report | `telemetry/tools/build_fw.py` |
| Bench test | `telemetry/BENCH_TEST.md` |

**Changes to the existing project:**
- All AT-command code is gone. The firmware sends nothing to the module except telemetry
  writes.
- USART2 changed from HAL at 9600 baud to LL at 115200 8N1, with one-shot DMA transmit on DMA1
  channel 4. The channel comes from the CubeMX F042 DMA database in CubeIDE 1.19. Receive uses
  the RXNE interrupt into a 128 B ring.
- HAL UART code and its MSP functions were removed.
- CubeIDE build configurations `Telemetry` (`TELEM_BAMO_DIAG=0`) and `Diag` (`=1`) were added:
  `-Os`, `-g3`.

## 2. Sizes (`arm-none-eabi-size`, GCC 13.3 from CubeIDE 1.19, `-Os`)

| Build | text | data | bss | Flash used | RAM (static + 0x200 heap + 0x400 stack) |
|---|---|---|---|---|---|
| Telemetry (flag 0) | 10728 | 12 | 3236 | **10,740 / 16,384 B (65.6%)** | 4,784 / 6,144 B |
| Diag (flag 1) | 13104 | 12 | 3700 | **13,116 / 16,384 B (80.1%)** | 5,248 / 6,144 B |
| Diag-first (flag 2) | 13036 | 12 | 3700 | 13,048 B (79.6%) | 5,248 B |

- CubeIDE's own headless build of `Telemetry` and `Diag` gives the same numbers: 0 errors,
  0 warnings.
- The binaries contain no `printf`, floating point or `malloc`.

## 3. Test results

```
$ py schema/gen_schema.py --check
gen_schema --check: generated files are up to date

$ py tests/host/run_host_tests.py        (zig cc, -Wall -Wextra -Werror)
== host tests, TELEM_BAMO_DIAG=0 ==   15 tests   864 checks, 0 failed
== host tests, TELEM_BAMO_DIAG=1 ==   23 tests  1112 checks, 0 failed
== host tests, TELEM_BAMO_DIAG=2 ==   21 tests   995 checks, 0 failed
host tests: all passed

$ py -m pytest tests
42 passed
```

### Host tests (section 8.1)

- **CRC:** test vector `"123456789"` → `0x29B1`.
- **Decoding:** every FAST source decodes with the right byte order. Example: 0x181
  `30 FF 7F 00` → speed 32767; BE brakes and BMS fields; LE Bamocar registers; 0xEB and 0x66
  both feed DC bus.
- **valid bits** clear exactly 1 ms after each timeout.
- **Events** fire on every edge in 3.6.
- **Event ring** drops the oldest, and the code-19 drop record reports the count.
- **Batch:** always 3 packets and 195 B, with header `0A 01 03` and deadlines at
  t0 + 200 ms × k. Packet 1 is the FAST from 100 ms earlier. The slot rotation follows 3.1 for
  all three flags.
- **Command validator, every rule in 5.6:**
  - all 7 blocked registers, via both BAMO_WRITE and SEND_FRAME
  - blocked reads of 0x30/0x49/0x4A, including through SEND_FRAME `3D ..`
  - all four live-inverter conditions and their time windows
  - the 3-read limit and the 20 frames/s rate limit
  - the 10 s session timeout sending `3D <reg> FF`
  - a repeated sequence number not run twice
  - TX_FAIL
- **Bad commands:** a bad CRC or bad key is ignored (no acknowledgement) and counted.

### Python tests (8.2–8.4)

- **CRC.**
- **Round trips:** encode → decode for every packet type and every command code, from the YAML.
- **Display scaling.**
- **Stream parser:** garbage, packets merged with `0A 01 03` between them, cut packets, and
  `Power on` / `channel Error` text split across reads.
- **Command sender** (fake clock and fake port): acknowledgement, resend after 5 batches, give-up
  after 10 tries, lockup pause and resume (both on `Power on` and after 90 s), no pause without
  missing packets, the 150 ms gap, PING keepalive and session close.
- **Logger → replay** produces identical CSVs.
- **CLI** command parsing.
- **Real-time `LoraLink` test** against a simulated car: the command goes out 108–130 ms after
  the batch arrives; 5 of 5 runs passed.
- **Cross-check (8.3):** a C program built against the generated header writes one packet of
  every type, every field filled with a known value. It also writes a real `telem_core` write
  built from section-3 CAN frames. Python decodes both with the YAML and checks every field.
  Example: rpm = 32767 / 32767 × Nmax 6000 = 6000; motor temp raw 9369 = 0 °C.
- **`gen_schema --check`.**

## 4. Changes from the brief, and interpretations

Changes made to fit or to work:

1. **USART2 and DMA use LL/register code, not HAL** (the brief allows this). HAL UART alone was
   ~2.5 KB.
   - The watchdog is register-level too. It is frozen while the debugger halts the core.
2. **The `.ioc` was edited by hand**, since CubeMX can't run here: USART2 115200 on LL, a
   USART2_TX DMA request on DMA1 channel 4, NVIC priorities (USART2 1, DMA 2, CAN 0), and
   `MX_DMA_Init`.
   - The generated-style code in `main.c` matches these settings.
   - **Open the `.ioc` in CubeMX once and check it before regenerating code.**
3. **The `Debug` configuration now builds with `-Og`** instead of `-O0`. At `-O0` the firmware
   overflows flash by 6,024 B, so the default configuration would not link. At `-Og` it builds
   (12,096 B, telemetry only).
   - Use `Telemetry` or `Diag` for the car.
   - The old committed `Debug/` output in git is stale; I didn't touch it.
4. **CAN controller settings, from `USER CODE` in `MX_CAN_Init`:**
   - Flag 0: silent mode (`BTR.SILM`).
   - Diag builds: automatic bus-off recovery on, and automatic retransmission **on**. The `.ioc`
     had it off, which drops a diagnostic frame that loses arbitration on a busy bus.

Interpretations where the brief was silent:

5. **Event edges:**
   - R2D and BSPD events fire only on transitions after the first frame (no event for the
     state at boot).
   - A BMS fault number other than 0 on the first frame counts as a change.
   - "Source came back" (15) fires the first time each source is seen, so boot produces one per
     present source.
   - Code 11 (inverter enable/disable command) fires when the 16-bit value changes, not on every
     frame, in case the GEVCU repeats it.
6. **Byte orders the brief doesn't give:**
   - 0x777 and 0x7E3 event values and the 0x481/0x666/0x321 raw fields are taken low byte = B0.
   - The command key goes on air as bytes `5A C3`, not as a little-endian u16.
7. **Diagnostics details:**
   - Check order: arguments → build allows → CAN ID → blocked register → inverter live →
     read limit → rate → send.
   - A BAMO_WRITE to register 0x3D (the read request) is BAD_ARGS, so it can't bypass the read
     rules. This is an added check.
   - A stop request (`period 0xFF`) while the inverter may be live is rejected, per "only
     BAMO_READ with period 0".
   - Stop frames sent at session end and by STOP_ALL are not rate-limited.
   - Frames over a capture slot's rate count as capture drops.
   - "Commands executed" counts commands with a new sequence number, whatever their status.
8. **Telemetry details:**
   - Packet sequence numbers are assigned when a packet goes into a write, so a refused write
     doesn't create a gap. The uptime stays the build time.
   - Missed write deadlines are skipped, not caught up.
   - `CAN frames received in the last 100 ms` uses a fixed 100 ms window, so flag 2 reports the
     same thing.
9. **PC side:**
   - The tool starts its command numbering after the car's last acknowledged sequence. A
     restarted PC tool could otherwise reuse the car's last number, and the car would treat it
     as a repeat.
   - Tries during a lockup don't count toward the 10.
   - The CLI sets capture slot 0 to 0x181 when a session starts, as the brief says.
   - Raw log times are epoch seconds.

## 5. Open issue: read replies are mostly lost (decision needed)

- Read replies only reach the PC through the capture of `0x181`. That ID also carries the
  GEVCU's own repeating reads (0x30, 0x49, 0x4A at 10 Hz each, about 30 frames/s).
- In the Diag build, DIAG_RESP gets at most 3 packet slots per second (9 frames/s). Those
  frames fill the capture's 20 frames/s limit and its 16-entry ring.
- The read itself is acknowledged OK; only the reply data is lost.

Simulation with the real `telem_core` + `diag_core`: a one-shot reply at a random time, 200
trials per row.

| Build | Reply reaches the PC (as built) | With the fix below |
|---|---|---|
| Diag (flag 1) | 42 / 200 (21%) | **200 / 200** |
| Diag-first (flag 2) | 123 / 200 (62%) | **200 / 200** |

- **Proposed fix:** don't capture 0x181 frames whose B0 is 0x30, 0x49 or 0x4A. FAST already
  carries them at 10 Hz. It is one line in `capture()` in `diag_core.c`.
- It changes rule 5.6.8 as written, so **it isn't applied**. Say the word and I'll add it with a
  host test.

## 6. Not done / limits

- **Bench acceptance (section 9)** needs the board and both modules: see `BENCH_TEST.md`. Nothing
  here was run on hardware. That includes the interactive CLI on a real port: its parsing and
  output are unit-tested, and `LoraLink` is tested with a simulated port.
- **GEVCU change (6.2):** not part of this task. `valid` bits 7 (KERN_STATUS) and 8 (DC bus) stay
  0 until it's made.
- **Scaling still TODO:** DC bus (Bamocar model unknown), BMS values, brakes, IGBT temp and
  actual current. The PC shows raw values.
- **The watchdog timeout depends on the LSI clock:** 40 kHz typical gives ~500 ms; the 30–50 kHz
  range gives ~400–670 ms.
- **Tools installed on this PC:** `pyyaml`, `pytest` and `ziglang` (a pip-installed C compiler
  for the host tests), in the user site-packages. Setting `CC=gcc` uses another compiler instead.

## 7. How to rerun

```
cd 2025-2026/Lora-Code/telemetry
py -m pip install -r pc/requirements.txt
py schema/gen_schema.py --check
py tests/host/run_host_tests.py
py -m pytest tests -q
py tools/build_fw.py --flags 0,1,2
```
