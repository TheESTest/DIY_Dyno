"""Read-only probe of whatever firmware is on the board.

Keeps DTR/RTS deasserted so opening the port does not reset the ESP32, and
sends only queries (READY?/STATUS) — nothing that commands the brake.
"""
import time

import serial

s = serial.Serial()
s.port = "/dev/ttyUSB0"
s.baudrate = 115200
s.timeout = 0.2
s.dsrdtr = False
s.rtscts = False
s.dtr = False
s.rts = False
s.open()
s.dtr = False
s.rts = False
time.sleep(1.5)
s.reset_input_buffer()

s.write(b"READY?\n")
s.write(b"STATUS\n")

deadline = time.time() + 4.0
data_seen = 0
first_data = None
others = []
while time.time() < deadline:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode("ascii", errors="replace").strip()
    if not line:
        continue
    if line.startswith("DATA,"):
        data_seen += 1
        if first_data is None:
            first_data = line
    else:
        others.append(line)

s.close()

print(f"DATA frames seen: {data_seen}")
if first_data:
    n = len(first_data.split(","))
    print(f"first DATA ({n} fields): {first_data}")
print("--- other lines ---")
for line in others[:25]:
    print(line)
