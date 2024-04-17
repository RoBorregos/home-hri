source scripts/args.sh

export ROS_IP=$LAPTOP_IP

# Start the speech processing

roslaunch speech speech.launch LAUNCH_DEVICES:=False SPEECH_PRE_PROCESSING:=True LAUNCH_STT:=True