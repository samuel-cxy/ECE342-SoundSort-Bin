import wave
import numpy as np
import matplotlib.pyplot as plt
import os

# 1. Ask the user for the file number
file_num = input("Enter the drop file number (Enter 0 for the first file): ")

# 2. Construct the filename dynamically based on the input
file_num = file_num.strip() # Remove any accidental spaces

if file_num == "0":
    filename = "stm32_mic_drop_item.wav"
else:
    filename = f"stm32_mic_drop_item ({file_num}).wav"

# 3. Check if the file exists before opening
if not os.path.exists(filename):
    print(f"Error: The file '{filename}' does not exist in this folder.")
else:
    with wave.open(filename, "rb") as wf:
        frames = wf.readframes(wf.getnframes())

    FS = 8000

    samples = np.frombuffer(frames, dtype=np.int16)

    print(f"--- Stats for {filename} ---")
    print("min:", samples.min(), "max:", samples.max(), "mean:", samples.mean())

    # Create an array of time values in seconds
    time_in_seconds = np.arange(len(samples)) / FS

    plt.plot(time_in_seconds, samples)
    plt.title(f"Waveform for {filename}")
    plt.xlabel("Time (Seconds)")
    plt.ylabel("Amplitude (16-bit PCM)")
    plt.show()