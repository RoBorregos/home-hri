# home-hri

This repository contains the developments of the Human-Robot Interaction area for RoBorregos team participating in RoboCup at Home OPL. Includes developments in `speech` using keyword-spotting, whisper, audio-processing and text-to-speech. The implementations of `natural language processing` include the use of fine-tunning OpenAI models, dataframe embeddings and conversational interactions. The ROS packages as well as the testing scripts with its dependencies are included.

## Docker setup

Using the makefile, run the commands:
```bash
# In root folder (Makefile in cwd)
make hri.build.cuda # You can run it without cuda, but Whisper runs very slow
bash docker/scripts/speech.bash # If you haven't before, run the speech script to create the pulseaudio socket

# Before creating the container, create and fill the .env file
# The .env.example file contanins instructions to select devices and filling variables

make hri.create.cuda # Create the container, could be without .cuda, depending on image
make hri.up # Start container
make hri.shell # Enter container

```
Inside the container

```bash
nvidia-smi # Optional: check if gpu is accessible
source ws/devel/setup.bash
```

## Speech
The `speech.launch` file starts the speech nodes for listening to the user, extracting the useful audio, processing it with whisper and calling the `say` node for the speaker.
```bash
# Run the package
roslaunch speech speech.launch
```

To test the speaker (and say.py)
```bash
make hri.shell # Enter docker in another terminal
rostopic pub /speech/speaker std_msgs/String "Hi, my name is Frida!" # Publish a message to robot_text topic
```

Test audio directly from terminal:
```bash
arecord -l # List input devices
aplay -l # List output devices

# Record audio. Select device with Dhw [card, device]
arecord -f cd -Dhw:1,7 -d 10 test.wav
aplay -Dhw:0,0 audio_file.wav

# If this error is present: 
# Warning: rate is not accurate (requested = 16000Hz, got = 48000Hz)
# please, try the plug plugin

# You can run:
aplay -D plughw:1,0 -r 16000 test4.wav
```

## Language processing
The `language_processing.launch` file executes the `command interpreter`, for extracting [Robot actions](https://github.com/RoBorregos/home/wiki/TMR-@HOME-2024#robot-actions-guide) from normal speech, and the `conversation` node for interacting with the user in a conversational manner:
```bash
roslaunch frida_language_processing language_processing.launch
```

To test the command interpreter, write text to the topic of `speech/raw_command`:
```bash
rostopic pub /speech/raw_command std_msgs/String "Grab an apple from the table, find Tony in the kitchen and give it to him"
```

To test the complete Human-Robot Interaction functionality, the speech nodes should be running.
