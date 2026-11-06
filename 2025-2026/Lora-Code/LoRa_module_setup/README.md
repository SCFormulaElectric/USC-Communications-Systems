# LoRa module setup tool (one-time)

The DX-LR02 module on the telemetry board is wired straight to the STM32 and can't be connected
to a PC. The old firmware configured it at every boot: 9600 baud and `AT+SF12`. The telemetry
firmware expects **115200 baud, MODE 1, LEVEL 7 (SF5), channel 03, MAC 0a,02** and sends no AT
commands. So a module left on the old settings never gets on the air.

This is a separate, tiny firmware (bare metal, CMSIS only; nothing shared with `LoRa_board`).
Flash it once, let it set the module up, then flash the telemetry firmware again.

## What it does

1. **Finds the baud rate.** Sends `+++` at 9600, 115200, 57600, 38400, 19200, 4800, 2400 and
   1200, up to 3 passes. `+++` toggles AT/data mode, so a module already in AT mode is handled.
2. **Saves the old settings.** `AT+HELP` goes to `g_before`.
3. **Writes the new settings.** `AT+MODE1`, `AT+LEVEL7` (sets SF5), `AT+CHANNEL03`,
   `AT+MAC0a,02`, `AT+POWE22`, and `AT+BAUD8` (115200) if needed, then `AT+RESET`.
   **It never sends `AT+SF12`.**
4. **Checks the result** at 115200 (`AT+HELP` and `AT+BAUD`), leaves AT mode, and tries the
   whole sequence once more if the check fails.
5. **Reports:**

   | LED (PA5) | Meaning |
   |---|---|
   | short flash about every 1–1.5 s | working: one flash per baud rate tried (up to ~40 s) |
   | **steady on** | **done**: module set up. A text report goes out over the radio every 3 s |
   | fast blink (10 Hz) | the module didn't answer at any baud rate: check its power and TX/RX wiring |
   | slow blink (1 Hz) | it answered, but the settings didn't check out: see `g_log` / `g_after` in the debugger |

   Everything is also in RAM for the debugger (Expressions view): `g_result` (1 OK, 2 no answer,
   3 check failed), `g_found_baud`, `g_before`, `g_after`, `g_log`.

## Steps

1. **Laptop:** plug in the PC-side module (MAC `0a,01`, e.g. COM8) and start the listener. Close
   anything else that has the port open first.
   ```
   cd 2025-2026\Lora-Code\LoRa_module_setup
   py listen.py COM8
   ```
2. **Flash the setup tool.** Either:
   - **CubeIDE:** *File → Import → General → Existing Projects into Workspace*, select the
     `LoRa_module_setup` folder, Finish. Then right-click the project → *Run As → STM32 C/C++
     Application*. Its one build configuration is `Setup`.
   - **Command line:** `py build.py --flash` (builds and flashes over ST-LINK).
3. **Wait for the LED to go steady** (10–40 s). The listener should then print, every 3 s:
   ```
   LORA MODULE SETUP OK. BEFORE: baud 9600 | MODE 1 | LEVEL 7 >> ... | SF 12 | MAC 0a,02 | ...
   AFTER: baud 115200 | MODE 1 | LEVEL 7 >> 13020.833984bps | SF 5 | MAC 0a,02 | freq 437200000hz >> 03 | power 22dBm | ...
   ```
   Those lines arriving prove the board's module is on the air with the right settings.
4. **Flash the telemetry firmware again** (`LoRa_board`, `Telemetry` configuration) and run
   `telemetry/pc/telemetry_cli.py --port COM8`. You should see 15.0 packets/s.

Don't leave this tool on the board: it only sends the report, not telemetry.

## Notes

- **The steady LED with no report on the laptop** means the board's module is set up, but its
  radio isn't reaching the PC module. Check that the PC module is `0a,01` on channel 03 at LEVEL 7
  (`tools/lora_framing_test/query_config.py COM8`), and the antennas.
- **Stray bytes while searching:** at a wrong baud rate the module may receive garbage and send a
  stray packet. This is harmless and stops once the rate is found.
- **What isn't changed:** the module's radio CRC setting (both modules were CRC off in the bench
  tests) and its sleep setting. Both are shown in the report.
- **Build:** 3.1 KB flash, 4.9 KB RAM (incl. stack and heap); CubeIDE headless build 0 errors,
  0 warnings.
