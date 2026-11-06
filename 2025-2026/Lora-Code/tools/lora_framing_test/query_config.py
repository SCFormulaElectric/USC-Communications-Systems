"""Read-only config query of both modules: detect baud, +++ into AT mode, AT+HELP, +++ out.
Usage: py query_config.py COM8 COM9 [outfile]
"""
import sys

from lora_common import LoRaModule

out = []
for port in sys.argv[1:3]:
    m = LoRaModule(port, port)
    baud = m.find_baud()
    out.append(f"=== {port}: baud {baud} ===")
    if baud is None:
        out.append("NO RESPONSE to +++ at any baud")
        m.close()
        continue
    m.set_at(True)
    out.append(m.send("AT+HELP", timeout=3.0).strip())
    out.append("AT+BAUD -> " + m.send("AT+BAUD").strip())
    m.set_at(False)
    m.close()

text = "\n".join(out)
print(text)
if len(sys.argv) > 3:
    open(sys.argv[3], "w").write(text + "\n")
