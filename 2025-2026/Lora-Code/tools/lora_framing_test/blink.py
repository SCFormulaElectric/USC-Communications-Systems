"""Make a port's adapter LEDs flicker so you can find it: enters AT mode and sends AT+BAUD
(a read-only query; nothing goes over the radio) about 4x/s, then leaves AT mode.
Usage: py blink.py COM8 [seconds]
"""
import sys
import time

from lora_common import LoRaModule

port = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20
m = LoRaModule(port, port)
m.set_at(True)
t_end = time.time() + secs
n = 0
while time.time() < t_end:
    m.ser.write(b"AT+BAUD\r\n")
    time.sleep(0.25)
    m.ser.reset_input_buffer()
    n += 1
m.set_at(False)
m.close()
print(f"{port}: sent {n} AT+BAUD queries, back in data mode")
