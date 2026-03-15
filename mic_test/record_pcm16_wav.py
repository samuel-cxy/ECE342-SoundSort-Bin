import serial
import wave
import time
import numpy as np
import os

PORT = "COM4"      # change this
BAUD = 460800
SECONDS = 2
FS = 8000

# Set your volume multiplier here!
# 1.0 is original volume, 5.0 is 5x louder, etc.
GAIN = 6.0 

# --- NEW AUTO-NAMING FUNCTION ---
def get_next_filename(base_name="stm32_mic", ext=".wav"):
    """Finds the next available filename to prevent overwriting."""
    if not os.path.exists(base_name + ext):
        return base_name + ext
    
    i = 1
    while True:
        new_name = f"{base_name} ({i}){ext}"
        if not os.path.exists(new_name):
            return new_name
        i += 1
# --------------------------------

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.set_buffer_size(rx_size=1048576) 
time.sleep(1)
ser.reset_input_buffer()

data = bytearray()
start = time.time()

print(f"Recording from STM32... (Volume Gain: {GAIN}x)")
while time.time() - start < SECONDS:
    chunk = ser.read(4096)
    if chunk:
        data.extend(chunk)

ser.close()

print("Processing audio packets...")
raw_bytes = bytes(data)

SYNC_HEADER = b'\xAA\xBB\xCC\xDD'
CHUNK_SIZE = 256  

chunks = raw_bytes.split(SYNC_HEADER)
valid_audio = bytearray()

for chunk in chunks[1:]:
    if len(chunk) >= CHUNK_SIZE:
        valid_audio.extend(chunk[:CHUNK_SIZE])

if len(valid_audio) > 0:
    samples = np.frombuffer(valid_audio, dtype=np.int16)

    # --- THE SAFE GAIN MULTIPLIER ---
    # 1. Convert to a larger float format so the math doesn't overflow
    samples_float = samples.astype(np.float32) * GAIN
    
    # 2. Clip the audio to the absolute 16-bit limits (-32768 to 32767)
    # This prevents integer wrapping (which causes static)
    samples_clipped = np.clip(samples_float, -32768, 32767)
    
    # 3. Convert back to standard 16-bit audio
    samples_final = samples_clipped.astype(np.int16)

    # Get a safe filename that won't overwrite your previous data
    save_filename = get_next_filename("stm32_mic_drop_item", ".wav")

    # Save to file using the new filename
    with wave.open(save_filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(FS)
        wf.writeframes(samples_final.tobytes())

    print(f"Success! Saved {len(samples)} perfectly aligned samples to '{save_filename}'.")
else:
    print("Error: No valid sync headers found. Check your C code and wiring.")