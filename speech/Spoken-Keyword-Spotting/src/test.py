import sounddevice as sd
import numpy as np

def record_audio(duration, samplerate, channels):
    print("Recording audio...")
    recording = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=channels, dtype='int16')
    sd.wait()
    print("Recording finished.")
    return recording

def play_audio(audio, samplerate):
    print("Playing audio...")
    sd.play(audio, samplerate)
    sd.wait()
    print("Playback finished.")

if __name__ == "__main__":
    duration = 5  # Duration of the recording in seconds
    samplerate = 44100  # Sample rate
    channels = 1  # Number of audio channels (1 for mono, 2 for stereo)

    # Record audio
    recorded_audio = record_audio(duration, samplerate, channels)

    # Play recorded audio
    play_audio(recorded_audio, samplerate)
