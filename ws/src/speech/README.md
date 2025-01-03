# Guide to run speech package


## 0. External roscore (optional)
Export the following variables in the terminal (Replace ROS_IP with your IP) and the ros master uri to the device running the roscore.

```bash
export ROS_MASTER_URI=http://192.168.31.23:11311
export ROS_IP=192.168.31.31
```


## 1. Create Docker

Using the makefile, run the commands:
```bash
# In root folder (Makefile in cwd)
# Create Image
make hri.build.cuda # You can run it without cuda, but Whisper runs very slow

# If you haven't before, run the speech script to create the pulseaudio socket
bash docker/scripts/speech.bash

# In the host machine, provide the needed permissions to allow docker to access devices using a non-root user
sudo usermod -aG audio $USER # Make sure current user has access to audio resources.
sudo chmod 777 /dev/snd/* # Allow access to audio devices.

# Before creating the container, create and fill the .env file
# The .env.example file contanins instructions to select devices and filling variables

# Create container
make hri.create.cuda # or without .cuda, depending on image

# Start container
make hri.up

# Enter container
make hri.shell

```

## 2. Inside the container

```bash
# Optional: check if gpu is accessible
ros@OAA:/workspace$ nvidia-smi # This command should output the gpu driver version

# Source the workspace
ros@OAA:/workspace$ source ws/devel/setup.bash

# Run the package
roslaunch speech speech.launch

# To test whisper, start talking and see the outputs in the terminal
```

## 3. To test the speaker (and say.py)
```bash
# Enter docker in another terminal
make hri.shell

# Publish a message to robot_text topic
rostopic pub /speech/speak_now std_msgs/String "Hi, my name is Frida!"

```


## 4. Test audio directly from terminal:
```bash
# List input devices
arecord -l 

# List output devices
aplay -l 
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

## For audio playback

It is best to use the pulse output device for audio playback, as it handles audio resampling, number of channels, among other things. When connecting and disconnecting devices, the default card associated with pulse may change.

In order to select a specific device for pulse:

1. Identify the card you want to select (ex: hw:1,0). You can use aplay and arecord with the plugin to make sure you are selecting the expected device.
2. List the devices with pacmd

```bash
# This command with output the available indices, the device with the * is the default device.
pacmd list-sinks
```
3. Set the new default device
```bash
pacmd set-default-sink <index>
```


## Helpful commands:

```bash
  scp <source> <dst> # Copy files from Jetson-host or host-jetson
  # Example
  scp nvidia@192.168.31.23:/home/nvidia/Music/test.wav  .
```

## ROS Nodes in `ws/src/speech/scripts`

### AudioCapturer.py
- **Purpose**: This node captures audio from the microphone.
- **Usage**: 
  - Publishes to: `rawAudioChunk`
  - Description: This node captures audio from the microphone and publishes it as raw audio chunks.

### Say.py
- **Purpose**: This node handles text-to-speech functionality.
- **Usage**: 
  - Subscribes to: `/speech/speak_now`
  - Provides service: `/speech/speak`
  - Description: This node converts text to speech using either an online or offline TTS engine.

### ReSpeaker.py
- **Purpose**: This node interfaces with the ReSpeaker hardware.
- **Usage**: 
  - Publishes to: `DOA`
  - Subscribes to: `ReSpeaker/light`
  - Description: This node interfaces with the ReSpeaker hardware to get the direction of arrival (DOA) of sound and control the LED ring.

### KWS.py
- **Purpose**: This node handles keyword spotting.
- **Usage**: 
  - Subscribes to: `rawAudioChunk`
  - Publishes to: `keyword_detected`
  - Description: This node uses Porcupine to detect keywords in the audio stream.

### UsefulAudio.py
- **Purpose**: This node processes useful audio segments.
- **Usage**: 
  - Subscribes to: `rawAudioChunk`, `saying`, `keyword_detected`
  - Publishes to: `UsefulAudio`
  - Description: This node processes audio segments to determine if they contain useful speech and publishes the useful audio.

### Hear.py
- **Purpose**: This node handles speech-to-text functionality.
- **Usage**: 
  - Subscribes to: `UsefulAudio`
  - Publishes to: `UsefulAudioAzure`, `UsefulAudioWhisper`
  - Description: This node converts speech to text using either an online or offline STT engine.

### Whisper.py
- **Purpose**: This node processes audio using the Whisper model.
- **Usage**: 
  - Subscribes to: `UsefulAudioWhisper`
  - Publishes to: `/speech/raw_command`
  - Provides service: `/speech/service/raw_command`
  - Description: This node uses the Whisper model to transcribe audio to text.
