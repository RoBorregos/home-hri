export JETSON_IP=192.168.31.123
export LAPTOP_IP=192.168.31.31
export XAVIER_IP=192.168.31.23

# Temporary ROS_IP (updated from the script that calls this one)
export ROS_IP=$LAPTOP_IP

export ROS_MASTER_URI=http://$XAVIER_IP:11311
