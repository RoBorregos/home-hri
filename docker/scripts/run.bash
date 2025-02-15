#!/bin/bash

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

VOLUME_COMMANDS="-v $PWD/ws:/workspace/ws"
for i in "$@"
do
case $i in
    # Receive ROS version from command line, through the --ros-distro argument
    --area=*)
    AREA="${i#*=}"
    echo "Home area set to: $AREA"
    shift # past argument=value
    ;;
    # Receive the --use-cuda argument
    --use-cuda)
    USE_CUDA=YES
    shift # past argument with no value
    ;;
    --jetson-l4t=*)
    JETSON_L4T="${i#*=}"
    shift # past argument=value
    ;;
    --is-display)
    IS_DISPLAY=YES
    shift # past argument with no value
    ;;
    --is-speech)
    IS_SPEECH=YES
    shift # past argument with no value
    ;;
    # Receive volumes to mount from command line, through the --volumes argument, each volume should be separated by a comma
    --volumes=*)
    for volume in $(echo ${i#*=} | tr "," "\n")
    do
        # If the path starts with ~, expand it to the user's home directory
        if [[ "$volume" == ~* ]]; then
            resolved_path="${volume/#\~/$HOME}"
        else
            resolved_path=$(realpath "$volume")
        fi
        folder_name=$(basename "$resolved_path")
        VOLUME_COMMANDS="$VOLUME_COMMANDS -v $resolved_path:/workspace/$folder_name"
    done
    shift # past argument=value
    ;;
    --name=*)
    # if the name is not empty, set the container name
    if [ -n "${i#*=}" ]; then
        CONTAINER_NAME="${i#*=}"
    fi
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

IMAGE_NAME="roborregos/home:$AREA-cpu"

if [ -z "$CONTAINER_NAME" ]; then
    CONTAINER_NAME="home-$AREA"
fi

# Additional commands required for each area (e.g. mounting volumes, setting environment variables, attaching devices, etc.)
# Note that more or different commands may be needed if using GPU or a Jetson device
ADDITIONAL_COMMANDS=""

# CHECK IF argument USE_CUDA is passed
DOCKER_GPU_ARGS=""
if [ -n "$JETSON_L4T" ]; then
    IMAGE_NAME="roborregos/home:$AREA-l4t-$JETSON_L4T"
    DOCKER_GPU_ARGS="--runtime nvidia"
    ADDITIONAL_COMMANDS+=""
    echo "Building for Nvidia Jetson with L4T: $JETSON_L4T"
fi

if [ -n "$IS_DISPLAY" ]; then
    IMAGE_NAME="roborregos/home:$AREA-cpu-display"
    DOCKER_GPU_ARGS=""
    ADDITIONAL_COMMANDS+=""
    echo "Building for display with cpu"
fi

if [ -n "$IS_SPEECH" ]; then
    DOCKER_SPEECH_ARGS="-v /tmp/pulseaudio.socket:/tmp/pulseaudio.socket -v /tmp/pulseaudio.client.conf:/etc/pulse/client.conf --device /dev/snd:/dev/snd"
    DOCKER_GPU_ARGS=""
    ADDITIONAL_COMMANDS+=""
    echo "Building for display with cpu"
fi

if [ -n "$USE_CUDA" ]; then
    IMAGE_NAME="roborregos/home:$AREA-cuda"
    DOCKER_GPU_ARGS="--gpus all"
    ADDITIONAL_COMMANDS+=""
    echo "Using CUDA"
fi

echo "Building docker image: $IMAGE_NAME"
echo "Container name: $CONTAINER_NAME"
echo "Volumes to mount: $VOLUME_COMMANDS"

DOCKER_COMMAND="docker run"

$DOCKER_COMMAND -it -d\
    $DOCKER_USER_ARGS \
    $DOCKER_GPU_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    $DOCKER_SPEECH_ARGS \
    $ADDITIONAL_COMMANDS \
    --privileged \
    --env-file .env \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /dev:/dev \
    --device /dev/video0:/dev/video0 \
    $VOLUME_COMMANDS \
    -w /workspace \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    bash