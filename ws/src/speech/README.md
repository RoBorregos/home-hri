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

Inside the container

```bash
# Optional: check if gpu is accessible
ros@OAA:/workspace$ nvidia-smi # This command should output the gpu driver version

# Source the workspace
ros@OAA:/workspace$ source ws/devel/setup.bash

# Run the package
roslaunch speech speech.launch

# To test whisper, start talking and see the outputs in the terminal
```

To test the speaker (and say.py)
```bash
# Enter docker in another terminal
make hri.shell

# Publish a message to robot_text topic
rostopic pub /robot_text std_msgs/String "Hi, my name is Frida!"

```


Test audio directly from terminal:
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

Helpful commands:

```bash
  scp <source> <dst> # Copy files from Jetson-host or host-jetson
  # Example
  scp nvidia@192.168.31.23:/home/nvidia/Music/test.wav  .
```