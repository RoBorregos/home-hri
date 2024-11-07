# List available devices:
# python3 -m sounddevice

from WavUtils import WavUtils
from SpeechApiUtils import SpeechApiUtils
import os

import sounddevice as sd


def get_devices():
    devices = sd.query_devices()
    num_dev = 0
    for device_info in devices:
        print(
            f"Device {num_dev}: {device_info['name']}, {device_info['max_input_channels']} input channels, {device_info['max_output_channels']} output channels")
        num_dev = num_dev + 1

    # print("Original order:")
    # print(devices)


if __name__ == "__main__":
    mp3_path = "play.mp3"
    WavUtils.play_mp3(mp3_path, device_index=12)
    # wav_path = "6_recorded_audio.wav"
    # wav_path = "test.wav"
    # WavUtils.play_mp3(mp3_path, device_index=12)
    # WavUtils.play_wav(wav_path, device_index=4)
    SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
    SPEAKER_INPUT_CHANNELS = int(
        os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
    SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

    OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(
        SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS)

    get_devices()
    print("OUTPUT_DEVICE_INDEX =", OUTPUT_DEVICE_INDEX)
