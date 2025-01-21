"""
Generate new models using: 
1. Simple (faster, less precise): https://colab.research.google.com/drive/1q1oe2zOyZp7UsB3jJiQ1IFn8z5YfjwEb?usp=sharing
2. More dev experience (slower, more precise): https://github.com/dscripka/openWakeWord/blob/main/notebooks/automatic_model_training.ipynb

"""

# Example script from: https://github.com/dscripka/openWakeWord/blob/main/examples/detect_from_microphone.py

# Imports
import pyaudio
import numpy as np
import openwakeword
from openwakeword.model import Model
import argparse

openwakeword.utils.download_models()

# Command to run model:
# python3 microphone.py --model_path ./Hey_Kahlo.onnx --inference_framework onnx

# Parse input arguments
parser=argparse.ArgumentParser()
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

args=parser.parse_args()

# Get microphone stream
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = args.chunk_size
audio = pyaudio.PyAudio()
mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

# Load pre-trained openwakeword models
if args.model_path != "":
    print(args.model_path)
    owwModel = Model(wakeword_models=[args.model_path], inference_framework=args.inference_framework)
else:
    owwModel = Model(inference_framework=args.inference_framework)

n_models = len(owwModel.models.keys())

# Run capture loop continuosly, checking for wakewords
if __name__ == "__main__":
    # Generate output string header
    print("\n\n")
    print("#"*100)
    print("Listening for wakewords...")
    print("#"*100)
    print("\n"*(n_models*3))

    while True:
        # Get audio
        audio = np.frombuffer(mic_stream.read(CHUNK), dtype=np.int16)

        # Feed to openWakeWord model
        prediction = owwModel.predict(audio)

        # Column titles
        n_spaces = 16
        output_string_header = """
            Model Name         | Score | Wakeword Status
            --------------------------------------
            """

        for mdl in owwModel.prediction_buffer.keys():
            # Add scores in formatted table
            scores = list(owwModel.prediction_buffer[mdl])
            curr_score = format(scores[-1], '.20f').replace("-", "")

            output_string_header += f"""{mdl}{" "*(n_spaces - len(mdl))}   | {curr_score[0:5]} | {"--"+" "*20 if scores[-1] <= 0.5 else "Wakeword Detected!"}
            """

        # Print results table
        print("\033[F"*(4*n_models+1))
        print(output_string_header, "                             ", end='\r')