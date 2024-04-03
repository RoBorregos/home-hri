import pyaudio
import wave

# Before testing the actual speech pipeline, test if audio is being recorded

def record_audio(output_file, duration=10, channels=1, sample_rate=48000, chunk_size=480, format=pyaudio.paInt16):
    audio = pyaudio.PyAudio()

    # Open the audio stream
    # See available devices with list_audio_devices() -> see AudioCapturer.py
    stream = audio.open(input_device_index=8,
                        format=format,
                        channels=channels,
                        rate=sample_rate,
                        input=True,
                        frames_per_buffer=chunk_size)

    print("Recording...")

    frames = []

    # Record audio data
    for _ in range(0, int(sample_rate / chunk_size * duration)):
        data = stream.read(chunk_size)
        frames.append(data)

    print("Recording stopped.")

    # Close and terminate the audio stream and PyAudio
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # Write audio data to a WAV file
    with wave.open(output_file, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(audio.get_sample_size(format))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))

    print(f"Audio recorded and saved to {output_file}")

if __name__ == "__main__":
    output_file = "recorded_audio.wav"
    record_audio(output_file, duration=10)