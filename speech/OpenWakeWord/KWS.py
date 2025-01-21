import pyaudio
import numpy as np
import openwakeword
from openwakeword.model import Model
import argparse
import time

# Initialize the download for the models
openwakeword.utils.download_models()

# Command to run model:
# python3 microphone.py --model_path ./Hey_Kahlo.onnx --inference_framework onnx

# Parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument(
    "--chunk_size",
    help="How much audio (in number of samples) to predict on at once",
    type=int,
    default=1280,
    required=False
)
parser.add_argument(
    "--model_path",
    help="The path of a specific model to load",
    type=str,
    default="",
    required=False
)
parser.add_argument(
    "--inference_framework",
    help="The inference framework to use (either 'onnx' or 'tflite'",
    type=str,
    default='tflite',
    required=False
)

args = parser.parse_args()

# Get microphone stream
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = args.chunk_size
audio = pyaudio.PyAudio()
mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

# Load pre-trained openwakeword models
if args.model_path != "":
    print(f"Loading model from {args.model_path}")
    owwModel = Model(wakeword_models=[args.model_path], inference_framework=args.inference_framework)
else:
    owwModel = Model(inference_framework=args.inference_framework)

n_models = len(owwModel.models.keys())

# Method to handle keyword detection and act upon it
def detect_and_respond(audio_data):
    prediction = owwModel.predict(audio_data)

    for mdl in owwModel.prediction_buffer.keys():
        scores = list(owwModel.prediction_buffer[mdl])
        curr_score = format(scores[-1], '.20f').replace("-", "")
        
        # Check if the wakeword is detected (score > 0.5 indicates detection)
        if scores[-1] > 0.5:
            print(f"Detected wakeword: {mdl} with score {curr_score[:5]}")
            handle_wakeword(mdl)  # Act upon the detected wakeword
            break

# Function to act upon the detected wakeword
def handle_wakeword(wakeword):
    # Switch-case-like structure
    if wakeword == "Hey_Kahlo":
        print("Activating 'Hey_Kahlo' specific action!")
    elif wakeword == "Hello_Google":
        print("Activating 'Hello_Google' specific action!")
    elif wakeword == "Frida":
        print("Activating 'Frida' specific action!")
    else:
        print(f"Wakeword '{wakeword}' detected, but no action defined.")

# Run capture loop continuously, checking for wakewords
if __name__ == "__main__":
    print("\n\n")
    print("#"*100)
    print("Listening for wakewords...")
    print("#"*100)
    print("\n"*(n_models * 3))

    try:
        while True:
            # Get audio from the microphone
            audio_data = np.frombuffer(mic_stream.read(CHUNK), dtype=np.int16)
            # Detect and respond to the wakeword
            detect_and_respond(audio_data)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping listening...")
        mic_stream.stop_stream()
        mic_stream.close()
        audio.terminate()
