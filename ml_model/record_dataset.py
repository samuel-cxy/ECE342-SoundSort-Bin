import serial
import wave
import os
import sys
import struct

PORT = "COM4"  # Change to your port
BAUD = 115200  
FS = 16000     

def get_next_filename(base_name="dataset_item", ext=".wav"):
    i = 1
    while os.path.exists(f"{base_name}_{i}{ext}"):
        i += 1
    return f"{base_name}_{i}{ext}"

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("Listening for STM32 hardware triggers... (Press Ctrl+C to stop)")
    ser.reset_input_buffer()
    
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line == "START_AUDIO":
            print("Trigger detected! Downloading text audio (takes ~4 seconds)...")
            audio_samples = []
            ser.timeout = 5.0 
            
            while len(audio_samples) < 8000:
                val_str = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if val_str == "END_AUDIO":
                    break
                    
                try:
                    val = int(val_str)
                    # THE FIX: Clamp the number to strictly 16-bit limits!
                    # If the text corrupts into a massive number, this saves the script.
                    val = max(-32768, min(32767, val)) 
                    audio_samples.append(val)
                except ValueError:
                    audio_samples.append(0)
            
            ser.timeout = 0.1 
            
            if len(audio_samples) > 0:
                filename = get_next_filename()
                with wave.open(filename, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(FS)
                    byte_data = struct.pack(f"<{len(audio_samples)}h", *audio_samples)
                    wf.writeframes(byte_data)
                    
                print(f"Success! Saved {len(audio_samples)} samples to -> {filename}\n")
            else:
                print("Error: No audio data received.\n")
                
            ser.reset_input_buffer() 

except KeyboardInterrupt:
    print("\nData collection stopped by user. Exiting cleanly...")
    
# THE FIX: Catch and print the exact error instead of hiding it!
except Exception as e:
    print(f"\n[!] SCRIPT CRASHED! The hidden error is: {e}")
    print("If it says 'Access is denied' or 'ClearCommError', the ST-Link USB crashed.")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")