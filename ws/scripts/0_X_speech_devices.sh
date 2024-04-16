source scripts/args.sh

export ROS_IP=$XAVIER_IP

# Start the speech devices

roslaunch speech speech.launch LAUNCH_DEVICES:=True SPEECH_PRE_PROCESSING:=False LAUNCH_STT:=False