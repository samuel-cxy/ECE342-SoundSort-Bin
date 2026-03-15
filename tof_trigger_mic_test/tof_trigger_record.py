import serial
import wave
import time
import numpy as np
import os

PORT = "COM4"      # change this
BAUD = 460800
SECONDS = 2
FS = 8000
GAIN = 8.0 

# Auto-naming function
def get_next_filename(base_name="stm32_mic_drop_item", ext=".wav"):
    if not os.path.exists(base_name + ext):
        return base_name + ext
    
    i = 1
    while True:
        new_name = f"{base_name} ({i}){ext}"
        if not os.path.exists(new_name):
            return new_name
        i += 1

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.set_buffer_size(rx_size=1048576) 
time.sleep(1)
ser.reset_input_buffer()

SYNC_HEADER = b'\xAA\xBB\xCC\xDD'
CHUNK_SIZE = 256  

print("--- Automated Dataset Collection Started ---")
print(f"Volume Gain: {GAIN}x | Sample Rate: {FS}Hz")

try:
    while True:
        print("\nWaiting for an object to fall through the ToF sensor... (Press Ctrl+C to stop)")
        
        # --- NEW: Wait for the distance message ---
        while True:
            if ser.in_waiting > 0:
                try:
                    # Read the text line sent by the STM32
                    line = ser.readline().decode('utf-8').strip()
                    
                    # Check if it's our distance message
                    if line.startswith("DIST:"):
                        dist_val = line.split(":")[1]
                        print(f"Trigger detected! Object distance: {dist_val} mm. Recording impact...")
                        break # Break out and start recording audio!
                        
                except UnicodeDecodeError:
                    # If we accidentally read binary audio, just ignore it
                    pass 
        # ------------------------------------------
        
        data = bytearray()
        start = time.time()

        # Record for exactly 2 seconds
        while time.time() - start < SECONDS:
            chunk = ser.read(4096)
            if chunk:
                data.extend(chunk)

        print("Processing audio packets...")
        raw_bytes = bytes(data)
        chunks = raw_bytes.split(SYNC_HEADER)
        valid_audio = bytearray()

        for chunk in chunks[1:]:
            if len(chunk) >= CHUNK_SIZE:
                valid_audio.extend(chunk[:CHUNK_SIZE])

        if len(valid_audio) > 0:
            samples = np.frombuffer(valid_audio, dtype=np.int16)

            # Apply safe gain multiplier
            samples_float = samples.astype(np.float32) * GAIN
            samples_clipped = np.clip(samples_float, -32768, 32767)
            samples_final = samples_clipped.astype(np.int16)

            # Get safe filename and save
            save_filename = get_next_filename("stm32_mic_drop_item", ".wav")
            with wave.open(save_filename, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(FS)
                wf.writeframes(samples_final.tobytes())

            print(f"Success! Saved perfectly aligned samples to '{save_filename}'.")
        else:
            print("Error: No valid sync headers found in this recording.")
            
        # VERY IMPORTANT: Flush the serial buffer before looping 
        # to ensure the next trigger is clean
        ser.reset_input_buffer()
        time.sleep(0.5) 

except KeyboardInterrupt:
    print("\nData collection stopped by user. Exiting cleanly...")

finally:
    ser.close()
    print("Serial port closed.")