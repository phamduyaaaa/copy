# copy
```bash
import serial
import time

# ================= Cấu hình =================
START_BYTE = 0xFF
STOP_BYTE  = 0xFE

# Chọn cổng UART trên Linux, ví dụ /dev/ttyUSB0
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    timeout=0.3
)

# ================= Dữ liệu gửi =================
vx = 0
vy = -2
az = 0
scale = 100.0

# Chuyển sang int16
vx_i = int(vx * scale)
vy_i = int(vy * scale)
az_i = int(az * scale)

# Giới hạn trong int16
vx_i = max(-32768, min(32767, vx_i))
vy_i = max(-32768, min(32767, vy_i))
az_i = max(-32768, min(32767, az_i))

# ================= Tạo frame =================
frame = bytearray()
frame.append(START_BYTE)

for v in (vx_i, vy_i, az_i):
    frame.append((v >> 8) & 0xFF)   # high byte
    frame.append(v & 0xFF)          # low byte

# Tính checksum XOR
checksum = frame[1]
for b in frame[2:8]:
    checksum ^= b
frame.append(checksum)
frame.append(STOP_BYTE)

# ================= Gửi UART =================
print("Frame sẵn sàng gửi:", frame)
ser.write(frame)
ser.flush()

# In hex cho debug
for i, b in enumerate(frame):
    print(f"Byte {i}: {hex(b)}")

ser.close()


```
