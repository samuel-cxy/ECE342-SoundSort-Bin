import serial
import wave
import time

PORT = "COM8"      # change this
BAUD = 921600
SECONDS = 10
FS = 16000

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

data = bytearray()
start = time.time()

print("Recording from ESP32...")
while time.time() - start < SECONDS:
    chunk = ser.read(4096)
    if chunk:
        data.extend(chunk)

ser.close()

# keep even number of bytes for int16
if len(data) % 2 != 0:
    data = data[:-1]

with wave.open("esp32_mic.wav", "wb") as wf:
    wf.setnchannels(1)
    wf.setsampwidth(2)   # 16-bit PCM
    wf.setframerate(FS)
    wf.writeframes(data)

print(f"Saved {len(data)} bytes to esp32_mic.wav")