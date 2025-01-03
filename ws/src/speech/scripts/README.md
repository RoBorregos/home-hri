# Speech pipeline

## AudioCapturer.py

Captures raw audio in chunks and publishes it.

- publish -> rawAudioChunk

## KWS.py

Uses porcupine to detect kew words sush as "Frida".

- subscribe -> rawAudioChunk
- publish -> keyword_detected

## UsefulAudio.py

Uses silero VAD to identify speech in raw audio and publish it to UsefulAudio.

- subscribe
    -> rawAudioChunk
    | saying
    | keyword_detected
- publish
    -> UsefulAudio
    | AudioState
    | colorInstruction
    | /ReSpeaker/light

## Hear.py

Takes UsefulAudio, performs STT with gRPC servers and publishes it.

- subscribe -> UsefulAudio
- publish -> /speech/transcription