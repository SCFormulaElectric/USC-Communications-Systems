# Bench acceptance test: LoRa telemetry and Bamocar diagnostics

Needs the LoRa board, both DX-LR02 modules, the car CAN bus (or a bench bus with the GEVCU, BMS
and Bamocar), and a PC. The host tests have already checked the logic. These steps check the real
radio, CAN and timing.

## 0. Setup (every session)

1. **Module configuration.** Check both modules without changing anything:
   ```
   py ../tools/lora_framing_test/query_config.py <car-module port> <PC-module port>
   ```
   Pass: both show `MODE:1`, `LEVEL:7`, `Spreading Factor:5`, `Coding rate:2`, `CRC:0`, channel
   `03`, power 22 dBm, `+BAUD=8` (115200). MACs: car `0a,02`, PC `0a,01`.
   The firmware sends no AT commands; if a setting is wrong, fix it by hand. **Never send
   `AT+SF12`.**
2. **Flash the board.** In STM32CubeIDE, build the `Telemetry` or `Diag` configuration.
   - Flash `Telemetry/LoRa_board.elf` or `Diag/LoRa_board.elf`, or use `telemetry/tools/build_fw.py`
     and flash `telemetry/tools/build/<flavour>/LoRa_board.bin`.
   - The existing `LoRa_board Debug.launch` points at the Debug build. Make a copy for
     Telemetry/Diag, or select their `.elf`.
3. **Check that it runs.** After power-up, the LED on PA5 starts blinking at 2.5 Hz after ~1 s
   (one toggle per write). A steady LED means it isn't writing: watchdog resets, or the board is
   stuck.
4. **Start the receiver:**
   ```
   cd telemetry/pc
   py telemetry_cli.py --port <PC-module port>
   ```

Record for every step: firmware build, ports, test start/end times, and the log folder.
`status` in the CLI prints all the counters named below.

## 1. Telemetry build, 10 minutes, no commands

Setup: `Telemetry` build (flag 0), CAN connected with normal traffic. Car powered but not driving
is fine. Don't type any commands.

Run 10 minutes, then `status` and `quit`.

| Check | Where | Pass |
|---|---|---|
| Packet rate | status line / `status` | **15.0 packets/s** (3 every 200 ms) |
| Lost packets | `status`: missing; `replay.py` on the raw log: "missing (seq gaps)" | **0** |
| Car module `channel Error` | `status`: car LoRa 'channel Error' (DIAG) | **0** |
| PC bad-CRC candidates | `status`: bad CRC | 0 (a few at start-up are OK) |
| CAN reception | `status`: CAN rx rising, overflow 0, bus-off 0 | overflow **0** |
| Silent mode | the car's other nodes see no error frames from the board; car TEC stays 0 | TEC **0** |
| Signals | `fast.csv`: speed, pack values, brakes change when the car does; `valid` bits set for the sources present | plausible |
| Build flag | `status`: `flag 0` | 0 |

Notes:
- `valid` bits 7 (KERN_STATUS) and 8 (DC bus) stay 0 until the GEVCU change (BUILD_INSTRUCTIONS
  6.2). That is expected.
- 0x473 (bit 11) only arrives while driving.

## 2. Diag build: session with a read every 2 s for 10 minutes

Setup: `Diag` build (flag 1). Inverter powered, **not enabled** (no R2D). The first command opens
a session.

1. `ping`, then `read N_MAX` and check the reply is printed (`N_MAX (0xC8) = ...`).
2. `quit` the CLI (it holds the port), then run the read loop for 10 minutes:
   ```
   py read_loop.py --port <PC-module port> --reg 0x25 --every 2 --minutes 10
   ```
   It prints the command results, link-state changes (lockups) and telemetry received/missing at
   the end.
3. Afterwards, `commands.csv` lists every attempt and its outcome; `diag_resp.csv` has the replies.

| Check | Where | Pass |
|---|---|---|
| Commands acknowledged | `commands.csv`: every `acked` has status 0 (OK) | **all**, except during a PC-module lockup |
| Gave up | `commands.csv`: `gave_up` rows | 0 outside lockups |
| Telemetry loss outside lockups | `status` / replay gap count, ignoring the lockup periods in `commands.csv` (`link_state` rows) | **0** |
| Car module `channel Error` | `status` (DIAG) | **0** |
| Bad commands on the car | `status`: commands bad | 0 |
| Lockups | `commands.csv` `link_state` rows: "PC radio restarting" then "OK"; the CLI prints `PC LoRa module printed 'Power on'` | note how many and how long (expected ~68.5 s each) |

During a lockup, expect ~20% telemetry loss and no acknowledgements. The tool pauses commands and
resumes on `Power on` (or after 90 s).

**Known limitation:**
- Read replies come back through the capture of `0x181`, which also carries the GEVCU's own
  repeating reads (~30 frames/s).
- In the Diag build a reply is forwarded only about 1 time in 5 (host simulation: 21%; 62% in
  the diagnostics-first build).
- The command itself is still acknowledged OK. See `IMPLEMENTATION_REPORT.md` for the proposed
  fix.
- So for this step, judge acknowledgements, not replies.

## 3. Safety

Setup: `Diag` build.

| Step | Command | Pass |
|---|---|---|
| a. Inverter powered, **not enabled** | `write 0x90 0` | `rejected: register blocked` (REJ_REGID) |
| b. Same | `write 0x51 0` | `rejected: register blocked` (REJ_REGID) |
| c. Same | `raw 0x201 90 00 00` | REJ_REGID (same rule via SEND_FRAME) |
| d. **R2D active** (inverter may be live) | `write 0x35 0` | `rejected: inverter may be live` (REJ_INV_ENABLED) |
| e. R2D active | `read 0xC8` (one-shot) | OK (allowed while live) |
| f. R2D active | `read 0xC8 --every 100` | REJ_INV_ENABLED |
| g. R2D off for 10 s, inverter disabled | `write 0x35 <original value>` | OK |

- Before step g, read the original value of 0x35 (`read 0x35`) so you can write it back. Writes
  only change RAM: an inverter power cycle restores the stored value.
- After the session (`stopall`, or 10 s without commands), check that no repeating read started
  by the PC is still running. Only the GEVCU's reads should remain on 0x181.
