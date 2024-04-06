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
aplay -D plughw:1,0 -r 16000 test.wav

```

Devices in Jetson Xavier


```bash
# Test Speaker
aplay -D plughw:0,0 test.wav
aplay -D plughw:0,0 -r 16000 test.wav

# Speaker:
card 0: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0


# Test microphone
arecord -Dhw:1,0 -r 44100 -d 10 test.wav

card 1: Device_1 [USB PnP Sound Device], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

Helpful commands:

```bash
  scp <source> <dst> # Copy files from Jetson-host or host-jetson
  # Example
  scp nvidia@192.168.31.23:/home/nvidia/Music/test.wav  .
```