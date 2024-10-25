# Running

## Install

```bash
sudo apt update
rosdep udpdate
rosdep install --from-paths src/ --ignore-src
```

## Networking

- Laptop

```bash
export ROS_IP=192.168.31.4
export ROS_MASTER_URI=http://192.168.31.105:11311

export ROS_IP=192.168.1.119
export ROS_MASTER_URI=http://192.168.1.119:11311
```

- Xavier
```bash
export ROS_IP=192.168.31.105
export ROS_MASTER_URI=http://192.168.31.105:11311
```

## Run a roscore on a tmux

```bash
tmux new -s roscore
# go to docker or ROS
roscore
```

To detach from tmux session:
type `ctrl-b + d` (in terminal).

## Recepcionist

- Initialize hri

```bash
# Enter docker
pwd # Root directory
make hri.up # Assuming container already exists
make hri.shell

# Build dependencies
pwd # /workspace/ws
catkin_make

# Source workspace
source devel/setup.bash

# Start recepcionist
roslaunch hri recepcionist_laptop.launch
roslaunch hri recepcionist_xavier.launch
```

## NLP debug

- Command interpreter is working

```bash
# Listen to commands interpreted
rostopic echo /task_manager/commands

# Listen to STT
rostopic echo /speech/raw_command

# Simulate user speech
rostopic pub /speech/raw_command std_msgs/String "data: 'Go to the kitchen and grab cookies'"
# Analyze outputs
```


## Speech debug

- Audio mic is working (ubuntu)

```bash
# List input devices
arecord -l 

# Record audio. 
# Select device with Dhw [card, device]
# -r -> rate of audio
# -c -> channel count
# -d -> duration of audio
arecord -Dhw:2,0 -f S16_LE -r 16000  -c 6 -d 10 test2.wav

```

- Speaker is working (ubuntu)

```bash
# List output devices
aplay -l 

# Play audio. 
# Select device with plughw [card, device]
# -r -> rate of audio
# -c -> channel count
# -d -> duration of audio
aplay -D plughw:1,0 -r 16000 test.wav

# Test speaker in xavier
play file.[mp3|wav]
```

- Audio mic is working (ros)
```bash
rostopic hz /rawAudioChunk
rostopic echo /rawAudioChunk
```

- Speaker is working (ros)
```bash
rostopic pub /speech/speak_now std_msgs/String "Hi, my name is Frida!"
```

## Common Errors and workaround

### OpenAI network

#### Traceback:
```bash
httpx.ConnectError: [Errno -3] Temporary failure in name resolution

File "/usr/local/lib/python3.8/dist-packages/openai/_base_client.py", line 960, in _request
    raise APIConnectionError(request=request) from err
openai.APIConnectionError: Connection error.
```

#### Solution:
```bash
sudo service docker restart
```

### Porcupine KWS:

#### Traceback:
```bash
    raise self._PICOVOICE_STATUS_TO_EXCEPTION[status] (
        pvporcupine._porcupine.PorcupineActivationLimitError
    )
```

#### Solution:
Replace API key for porcupine, or use device used previously with key.
