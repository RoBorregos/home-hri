# Ensure dependencies are installed
# pip install --upgrade git+https://github.com/UKPLab/sentence-transformers
# pip install keybert ctransformers
# pip install --upgrade git+https://github.com/huggingface/transformers
# pip install vosk
# pip install sounddevice

import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer
from ctransformers import AutoModelForCausalLM
from transformers import AutoTokenizer, pipeline
from keybert import KeyBERT

# Load model
model = AutoModelForCausalLM.from_pretrained(
    "TheBloke/Mistral-7B-Instruct-v0.1-GGUF",
    model_file="mistral-7b-instruct-v0.1.Q4_K_M.gguf",
    model_type="mistral",
    gpu_layers=50,
    hf=True
)

# Load tokenizer
tokenizer = AutoTokenizer.from_pretrained("mistralai/Mistral-7B-Instruct-v0.1")

# Initialize KeyBERT
kw_model = KeyBERT(model)

# Initialize Vosk model for offline speech recognition
vosk_model = Model("speech/KeyBART/vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(vosk_model, 16000)

# Queue to hold audio data
q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# Extract keywords from text
def extract_keywords(text):
    keywords = kw_model.extract_keywords(text, keyphrase_ngram_range=(1, 2), stop_words=None)
    return keywords

# Real-time speech-to-text and keyword extraction
with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                       channels=1, callback=callback):
    print("Listening...")
    while True:
        data = q.get()
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            result_dict = json.loads(result)
            text = result_dict.get("text", "")
            if text:
                print(f"Transcribed Text: {text}")
                
                # Extract keywords
                keywords = extract_keywords(text)
                print(f"Keywords: {keywords}")