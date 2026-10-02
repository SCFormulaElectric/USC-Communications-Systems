# CAN Viewer

Tkinter GUI that shows live CAN traffic and decodes each frame with a DBC file:
message names, signal values with units, enum states, and inverter fault bits.
Runs on Windows, macOS and Linux.

## Setup

```
pip install -r requirements.txt
python can_viewer.py
```

## Which DBC

Our inverter is a Cascadia CM-series with a non-oil-cooled motor, so load:

**`20250206_CM_not_oil-cooled_CAN_DB.dbc`**

Load only one DBC. The Cascadia files reuse the same IDs. See
`DBC File Use Guidance.txt` for the other variants.

## Reading the car (STM32 Nucleo + CAN transceiver)

The Nucleo firmware prints each frame over its USB COM port as text:

```
ID: 0x0A3 STD DLC: 8 Data: F3 C5 35 1F 10 00 00 00
```

1. Plug in the Nucleo and close anything else using its COM port
   (CubeIDE serial console, PuTTY, ...).
2. **Load DBC...** and pick the file above.
3. Interface: `serial text (STM32)`. Channel is filled in automatically (e.g. `COM8`).
4. **Connect**.

The UART baud defaults to 115200. If the firmware uses a different one, type
it after the port, e.g. `COM8@921600`. At 115200 the serial link tops out
around 200 frames/s, which is less than the inverter sends, so some frames are
dropped. Raising the firmware UART baud fixes this.

The **Bitrate** box only feeds the bus-load % in the status bar. The real CAN
bitrate is set in the STM32 firmware.

Other adapters (PCAN, Kvaser, CANable/slcan, gs_usb, Vector, IXXAT, socketcan)
work through python-can. Install the vendor driver, pick the interface, and
connect.

## Without the car

- **Replay log...** → `rotation.asc`: a 33 s recording from the car
  with the motor turned slowly (up to ~18 rpm).
- Interface `demo (simulated)` generates fake traffic from the loaded DBC.

## The display

- **Live tab**: Node → Message → Signal tree, one row per ID updated in place.
  Each message gets its own colour. Blue = value just changed, red = out of range
  or fault active, grey = stale (stopped arriving). IDs not in the DBC are
  listed under "Not in DBC".
- **Trace tab**: every frame in time order.
- **Details pane**: click a row for bit layout, scaling, range, value table,
  DBC comment, and active fault bits.
- **Record...** saves the session to .asc / .blf / .csv / .log.
- **Filter**: search by ID, message, or signal name.

Fault bits show as numbers until their names are filled into
`fault_bits.json` (from the Cascadia manual's fault code table).
