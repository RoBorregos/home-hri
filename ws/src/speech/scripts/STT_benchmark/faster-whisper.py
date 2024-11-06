import os
import time
from faster_whisper import WhisperModel
import torch

# Load the model (on CUDA if available)
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")
# device = "cpu"

model = WhisperModel("base", device=device, compute_type="float32")

# Specify the directory containing the audio files
recordings_dir = "recordings"

# Iterate through all files in the recordings subdirectory
for filename in os.listdir(recordings_dir):
    if filename.endswith(".wav"):  # Only process .wav files
        file_path = os.path.join(recordings_dir, filename)

        print(f"Transcribing file: {filename}")

        start = time.time()

        # Transcribe the audio file in Spanish (use "en" for English)
        segments, info = model.transcribe(
            file_path,
            language="en",
            hotwords="Frida kitchen attendance",
            condition_on_previous_text=True,
        )  # Change "es" to "en" for English

        # Print transcription results
        for segment in segments:
            print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

        # Print time taken for each file
        print(f"Time taken for {filename}: {time.time() - start:.2f} seconds\n")
