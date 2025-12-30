# copy
```bash
import serial
import msvcrt
import time
import keyboard   # pip install keyboard

START_BYTE = 0xFF
STOP_BYTE  = 0xFE
ser = serial.Serial(
    port='COM12',
    baudrate=115200,
    timeout=0.3
)
vx = 0
vy = -2
az = 0


scale = 100.0

vx_i = int(vx * scale)
vy_i = int(vy * scale)
az_i = int(az * scale)

vx_i = max(-32768, min(32767, vx_i))
vy_i = max(-32768, min(32767, vy_i))
az_i = max(-32768, min(32767, az_i))

frame = bytearray()
frame.append(START_BYTE)

for v in (vx_i, vy_i, az_i):
    frame.append((v >> 8) & 0xff)
    frame.append( v       & 0xff)

checksum = frame[1]
for b in frame[2:8]:
    checksum ^= b
frame.append(checksum)
frame.append(STOP_BYTE)

print(frame)
ser.write(frame)
for i in frame:
    print(hex(i))



```
