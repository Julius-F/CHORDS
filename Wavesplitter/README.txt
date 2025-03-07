WAVESPLITTER.PY - WAV File to CSV Converter

Wavesplitter.py is a Python script that extracts sample data from a .wav file and generates a properly formatted CSV file. It allows users to:

✔ Select a .wav file using a file dialog
✔ Display the name of the selected .wav file in the command prompt
✔ Choose between Left (L), Right (R), or Stereo (S) channels
✔ Scale the WAVETABLE_SIZE by resampling the waveform
✔ Convert the variable type after rescaling for correct data representation
✔ Choose the output variable type: int, uint, or float
✔ Extract minimum and maximum sample values
✔ Estimate bit depth and provide theoretical signed/unsigned ranges
✔ Ask the user for a filename, or default to "WaveTable YYYY-MM-DD HH-MM-SS.csv"
✔ Output a CSV file with a structured header and waveform data

INSTALLATION & DEPENDENCIES
To ensure you have all the required dependencies, open Command Prompt (CMD) and copy-paste the following command:

pip install numpy scipy tk
This installs:

numpy → For handling waveform data as arrays
scipy → For resampling the waveform when scaling
tk (Tkinter) → For opening the file selection dialog

HOW TO USE
Run the script:

python wavesplitter.py
A file dialog will appear → Select a .wav file. The selected filename will be displayed in the command prompt.

Choose the audio channel:

Type l → Left Channel (Default)
Type r → Right Channel
Type s → Keep Stereo
Press Enter → Defaults to Left
Choose to scale WAVETABLE_SIZE:

Enter a new size → Rescales waveform data
Press Enter → Keeps original size
Choose the variable type:

Type i (default) → Integer (signed)
Type u → Unsigned Integer (absolute values)
Type f → Float (-1.0 to 1.0 range)
Press Enter → Defaults to Integer
Enter a filename (without extension) or press Enter to use default:

If a custom filename is entered, the file will be saved as YourName.csv.
If Enter is pressed, the file will be saved as "WaveTable YYYY-MM-DD HH-MM-SS.csv".
The script processes the waveform and outputs a CSV file.

CSV FILE STRUCTURE
The generated file will be named:

WaveTable YYYY-MM-DD HH-MM-SS.csv
or

YourCustomName.csv

Example CSV header:

WAVETABLE_SIZE 1024
VARIABLE_TYPE int
MIN_SAMPLE -32767, MAX_SAMPLE 32767
ESTIMATED_BIT_DEPTH 16
BIT_DEPTH_MIN -32768, BIT_DEPTH_MAX 32767
BIT_DEPTH_U_MIN 0, BIT_DEPTH_U_MAX 65535

After the header, the waveform sample data is written in a single line, comma-separated:

100, -120, 354, -500, 1024, ...

NOTES
The script displays the name of the selected .wav file in CMD.
If a stereo file is selected, the script will prompt you to choose a channel.
If resampling is chosen, the waveform is adjusted to fit the new size.
If no .wav file is selected, the script will exit automatically.
Variable type conversion happens AFTER resampling to ensure correct representation.
All timestamps are based on the system clock at runtime.

SUPPORT
If you encounter issues, ensure:

Python is installed (Download from: https://www.python.org/)

You have installed the required dependencies using:

python -m pip install numpy scipy tk
The .wav file is in PCM format (not compressed like MP3 or FLAC)

🎵 Enjoy using Wavesplitter.py! 🎵

Written By:

Bjorn Lavik, Sonoma State University, 2025 for CHORDS Senior Design Project
