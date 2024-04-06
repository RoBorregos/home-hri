## Speech

Test audio:
```bash
# List devices
arecord -l 
# Record audio. Select device with Dhw [card, device]
arecord -f cd -Dhw:1,7 -d 10 test.wav

aplay -Dhw:0,0 audio_file.wav

# If this error is present: 
# Warning: rate is not accurate (requested = 16000Hz, got = 48000Hz)
# please, try the plug plugin

# You can run:
aplay -D plughw:1,0 -r 16000 test4.wav

```