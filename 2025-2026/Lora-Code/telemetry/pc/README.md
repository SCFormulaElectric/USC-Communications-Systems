# PC tools: LoRa telemetry receiver and Bamocar diagnostics console

Python 3.10+. Install once:

```
py -m pip install -r requirements.txt
```

The PC-side DX-LR02 module must already be configured (MODE1, LEVEL7, channel 03, MAC 0a,01,
115200 baud). These tools never send AT commands.

## Run

```
py telemetry_cli.py --port COM9                 # status line + diagnostics prompt
py telemetry_cli.py --port COM9 --watch         # status line only (Ctrl+C to quit)
py telemetry_cli.py --port COM9 --delay-ms 110  # command delay after a batch (default 110)
```

The status line updates twice a second:

```
OK       15.0 pkt/s loss  0.0% |   6000 rpm | tq  12.3% | pack V 3456 I 120 SOC 85 | cell 32C | brk 120/118 | bspd_ok r2d | valid 0x7E7F
```

- Speed in rpm needs Nmax, which comes from the DIAG packet (sent once per second). Until
  then the line shows the raw value.
- BMS values and DC bus voltage are raw: their scaling isn't decided yet (TODO in the schema).
- `valid` is shown as hex only when some sources are missing.
- Link state is `OK`, `no data` (nothing for 2 s), or `PC radio restarting, commands paused`.
  In the last case the PC module has locked up; commands resume on its `Power on`, or after 90 s.

### Diagnostics commands (Diag build on the car)

| Command | What it does |
|---|---|
| `ping` | start / keep a session (the tool also PINGs every 3 s while a session is open) |
| `read <reg> [--every <ms>]` | Bamocar read, once or every 20–254 ms. The reply is shown by register name |
| `stop <reg>` | stop a repeating read |
| `write <reg> <value> [--32]` | Bamocar parameter write (RAM only; lost at inverter power-off) |
| `raw <id> <bytes...> [--ext]` | send a CAN frame. IDs above 0x7FF are extended |
| `capture <id> [<mask>] [--slot N] [--rate N]` | forward matching CAN frames (slot 0–3, default rate 20/s) |
| `uncapture [<slot>]` | clear one slot, or all |
| `stopall` | stop reads, clear captures, end the session |
| `status` | link, command and car counters |
| `quit` | sends STOP_ALL first if a session is open |

- Registers can be numbers (`0xC8`) or names (`N_MAX`). The names are in
  `../schema/telemetry_schema.yaml` (`bamocar_registers`).
- Every command prints its acknowledgement in words, e.g. `write TORQUE_SETPOINT (0x90) = 1
  (16-bit) -> rejected: register blocked`.
- The first command of a session also sets capture slot 0 to `0x181`, so Bamocar replies come
  back.
- `read 0x8F` asks before sending: the GEVCU sends an error clear when it sees that reply with
  byte 1 = 0x20.
- The car enforces the safety rules (blocked registers, inverter-live checks, limits). The PC
  only reports the result.

## Logs and CSVs

Each run writes to `--log-dir` (default `logs/`):

| File | Contents |
|---|---|
| `raw_<stamp>.bin` | **master record**: every good packet as `[f64 PC time, epoch s][64 B packet]` |
| `<stamp>/fast.csv` | one row per FAST packet (10 Hz). Raw fields, plus scaled columns like `speed_rpm`, `motor_temp_degC`, `torque_cmd_%`, `torque_req_Nm`, and `valid_names` / `status_names` |
| `<stamp>/therm.csv` | 0x1839F380–385 data bytes as hex (contents not confirmed yet) |
| `<stamp>/diag.csv` | car counters, Bamocar scaling registers, firmware version, build flag |
| `<stamp>/events.csv` | one row per event: `event_time_ms` is car uptime (packet uptime − age) |
| `<stamp>/diag_resp.csv` | acknowledgements (`kind=ack`) and captured frames (`kind=frame`, with `bamo_reg`/`bamo_value` for 0x181) |
| `<stamp>/commands.csv` | every command attempt (`send`, `resend`), its outcome (`acked` with status, or `gave_up`), and link-state changes |

- Columns common to all CSVs: `pc_time` (epoch seconds), `seq` (packet sequence number),
  `uptime_ms` (car uptime when the packet was built).
- Gaps in `seq` are lost packets.

Rebuild the CSVs from a raw log (everything except `commands.csv`):

```
py replay.py logs/raw_20261105_201500.bin
```

## Files

- `decoder.py`: loads the YAML; decode, encode and display scaling
- `lora_link.py`: serial port, stream parser, batch timing, command sender, lockup handling
- `logger.py`: raw log and CSV logging
- `replay.py`: rebuilds the CSVs from a raw log
- `telemetry_cli.py`: the console
- `read_loop.py`: bench test 2 helper (a read every N s for M minutes, with a summary)

## Tests

```
cd ..
py -m pytest tests -q                   # Python tests (also builds and runs the C cross-check)
py tests/host/run_host_tests.py         # firmware logic tests in C (flags 0, 1, 2)
py schema/gen_schema.py --check         # generated header up to date
```
