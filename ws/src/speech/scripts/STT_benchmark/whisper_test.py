import os
import whisper
import time
import torch

# Check if CUDA is available and load the model on the appropriate device
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load the Whisper model on GPU (if available)
model = whisper.load_model("base", device=device)

# Specify the directory containing the audio files
recordings_dir = "recordings"

# Iterate through all files in the recordings subdirectory
for filename in os.listdir(recordings_dir):
    if filename.endswith(".wav"):  # Only process .wav files
        file_path = os.path.join(recordings_dir, filename)

        print(f"Transcribing file: {filename}")

        start = time.time()

        # Transcribe the audio file in Spanish (use "en" for English)
        result = model.transcribe(
            file_path,
            language="en",
            condition_on_previous_text=True,
        )  # Change "es" to "en" for English

        # Print transcription results
        print("Transcription:", result["text"])

        # Print time taken for each file
        print(f"Time taken for {filename}: {time.time() - start:.2f} seconds\n")
