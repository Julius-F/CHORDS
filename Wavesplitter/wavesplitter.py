import wave
import numpy as np
import datetime
import os
import tkinter as tk
from tkinter import filedialog
from scipy.signal import resample

# Create a file dialog to let the user select a .wav file
root = tk.Tk()
root.withdraw()  # Hide the root window
file_path = filedialog.askopenfilename(title="Select a WAV file", filetypes=[("WAV files", "*.wav")])

# Ensure the user selected a file
if not file_path:
    print("No file selected. Exiting...")
    exit()

# Extract just the file name for display
file_name = os.path.basename(file_path)
print(f"Selected file: {file_name}")

# Open the selected .wav file
with wave.open(file_path, 'rb') as wav_file:
    # Get parameters
    num_channels = wav_file.getnchannels()
    sample_width = wav_file.getsampwidth()  # Bytes per sample
    frame_rate = wav_file.getframerate()
    num_frames = wav_file.getnframes()

    # Read frames and convert to numpy array
    frames = wav_file.readframes(num_frames)
    dtype_map = {1: np.uint8, 2: np.int16, 3: np.int32, 4: np.int32}  # 24-bit WAVs stored in 32-bit containers
    dtype = dtype_map.get(sample_width, np.int16)  # Default to 16-bit if unknown
    audio_data = np.frombuffer(frames, dtype=dtype)

    # If stereo, reshape array to separate channels
    if num_channels > 1:
        audio_data = audio_data.reshape(-1, num_channels)

# Handle stereo selection
if num_channels > 1:
    choice = input("Choose channel - Left (l), Right (r), Stereo (s) [Default: Left]: ").strip().lower()
    if choice == "r":
        audio_data = audio_data[:, 1]  # Take the right channel
        print("Using Right Channel.")
    elif choice == "s":
        print("Using Stereo (both channels kept).")
    else:
        audio_data = audio_data[:, 0]  # Default to left channel
        print("Using Left Channel (Default).")

# Ask the user if they want to scale the WAVETABLE_SIZE
original_size = len(audio_data)
wavetable_size = original_size  # Default to the original size

size_input = input(f"Current WAVETABLE_SIZE is {original_size}. Enter new size to scale, or press Enter to keep it: ").strip()
if size_input.isdigit():
    new_size = int(size_input)
    if new_size > 0 and new_size != original_size:
        audio_data = resample(audio_data, new_size)  # Resample to the new wavetable size
        wavetable_size = new_size
        print(f"Rescaled WAVETABLE_SIZE to {wavetable_size}.")
    else:
        print("Invalid size, keeping original size.")
else:
    print("Keeping original WAVETABLE_SIZE.")

# Ask user for variable type
var_type_input = input("Choose variable type - int (i), uint (u), float (f) [Default: int]: ").strip().lower()

# Convert variable type AFTER rescaling
if var_type_input == "u":
    variable_type = "uint"
    audio_data = np.abs(audio_data).astype(np.uint32)  # Convert to unsigned integers
elif var_type_input == "f":
    variable_type = "float"
    audio_data = audio_data.astype(np.float32) / np.max(np.abs(audio_data))  # Normalize to -1.0 to 1.0 range
else:
    variable_type = "int"  # Default to integer
    audio_data = audio_data.astype(int)

print(f"Using Variable Type: {variable_type}")

# Get min and max sample values
min_sample = np.min(audio_data)
max_sample = np.max(audio_data)

# Estimate bit depth
bit_depth_map = {1: 8, 2: 16, 3: 24, 4: 32}
bit_depth = bit_depth_map.get(sample_width, 16)  # Default to 16-bit

# Calculate theoretical min/max for the estimated bit depth (signed representation)
bit_depth_min = -(2**(bit_depth - 1))
bit_depth_max = (2**(bit_depth - 1)) - 1

# Calculate unsigned integer representation range
bit_depth_u_min = 0
bit_depth_u_max = (2**bit_depth) - 1  # e.g., 16-bit -> 2^16 - 1

# Ask the user for a custom filename
timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H-%M-%S")
default_filename = f"WaveTable {timestamp}.csv"
user_filename = input(f"Enter a filename (without extension) or press Enter to use default [{default_filename}]: ").strip()

# Set the final filename
if user_filename:
    output_filename = f"{user_filename}.csv"
else:
    output_filename = default_filename

# Create 'Outputs' folder if it doesn't exist
output_folder = "Outputs"
os.makedirs(output_folder, exist_ok=True)

# Set the full output file path
output_path = os.path.join(output_folder, output_filename)

print(f"Output file: {output_path}")

# Save sample data to a CSV file in the Outputs folder
with open(output_path, "w") as file:
    # Write header information
    file.write(f"WAVETABLE_SIZE {wavetable_size}\n")
    file.write(f"VARIABLE_TYPE {variable_type}\n")
    file.write(f"MIN_SAMPLE {min_sample}, MAX_SAMPLE {max_sample}\n")
    file.write(f"ESTIMATED_BIT_DEPTH {bit_depth}\n")
    file.write(f"BIT_DEPTH_MIN {bit_depth_min}, BIT_DEPTH_MAX {bit_depth_max}\n")
    file.write(f"BIT_DEPTH_U_MIN {bit_depth_u_min}, BIT_DEPTH_U_MAX {bit_depth_u_max}\n")

    # Write sample data as a single line of comma-separated values
    file.write(", ".join(map(str, audio_data)))

print(f"Sample data written to {output_path} with WAVETABLE_SIZE {wavetable_size}")
print(f"Variable Type: {variable_type}")
print(f"Min Sample: {min_sample}, Max Sample: {max_sample}")
print(f"Estimated Bit Depth: {bit_depth} (Signed Range: {bit_depth_min} to {bit_depth_max})")
print(f"Unsigned Range: {bit_depth_u_min} to {bit_depth_u_max}")
