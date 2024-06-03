import argparse
import logging
import sys
import time
import wave
from pathlib import Path
from typing import Any, Dict

from piper import PiperVoice
from download import ensure_voice_exists, find_voice, get_voices

from WavUtils import WavUtils

_FILE = Path(__file__)
_DIR = _FILE.parent
_LOGGER = logging.getLogger(_FILE.stem)


MODEL = "en_US-amy-medium"
OUTPUT_FILE = "output.wav"

OUTPUT_TEXT = "Hello, my name is Frida. How can I help you today?"
DOWNLOAD = Path.cwd()

# def split_text(text: str, max_len):
#     text_chunks = text.split(".")
#     limited_chunks = []
#     for chunk in text_chunks:
#         while len(chunk) > 0:
#             limited_chunks.append(chunk[:max_len])
#             chunk = chunk[max_len:]

#     return limited_chunks
    

def main() -> None:
        DEBUG = True
        logging.basicConfig(level=logging.DEBUG if DEBUG else logging.INFO)

        model_path = Path(MODEL)
        if not model_path.exists():
            # Load voice info
            voices_info = get_voices(DOWNLOAD, update_voices=True)

            # Resolve aliases for backwards compatibility with old voice names
            aliases_info: Dict[str, Any] = {}
            for voice_info in voices_info.values():
                for voice_alias in voice_info.get("aliases", []):
                    aliases_info[voice_alias] = {"_is_alias": True, **voice_info}

            voices_info.update(aliases_info)
            ensure_voice_exists(MODEL, [DOWNLOAD], DOWNLOAD, voices_info)
            model_path, model_config = find_voice(MODEL, [DOWNLOAD])

        # Load voice
        voice = PiperVoice.load(model_path, config_path=model_config, use_cuda=False)
        synthesize_args = {
            "speaker_id": None,
            "length_scale": None,
            "noise_scale": None,
            "noise_w": None,
            "sentence_silence": 0.0,
        }

        if OUTPUT_FILE:
            text = OUTPUT_TEXT
            with wave.open(OUTPUT_FILE, "wb") as wav_file:
                voice.synthesize(text, wav_file, **synthesize_args)

# def generate_wav(output_base_path, text_chunks):
#     for text in text_chunks:
#         output_path = output_base_path / f"{text}.wav"
#         with wave.open(output_path, "wb") as wav_file:
#             voice.synthesize(text, wav_file, **synthesize_args)

# def say(text: str):
#     audio_paths = generate_wav(split_text(text, 100))
    
#     for audio_path in audio_paths:
#         WavUtils.play_wav(audio_path)
#         WavUtils.discard_wav(audio_path)
    

if __name__ == "__main__":
    main()