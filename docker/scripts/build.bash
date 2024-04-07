#!/bin/bash
# Use Dockerfile according to NVIDIA GPU and CUDA availability

# Receive ROS version from command line, through the --ros-distro argument
for i in "$@"
do
case $i in
    --area=*)
    AREA="${i#*=}"
    shift # past argument=value
    ;;
    --use-cuda)
    USE_CUDA=YES
    shift # past argument with no value
    ;;
    --jetson-l4t=*)
    JETSON_L4T="${i#*=}"
    shift # past argument=value
    ;;
    *)
    # unknown option
    ;;
esac
done

# if ros-distro not set, return error
if [ -z "$AREA" ]; then
    echo "ROS distro not set. Use --ros-distro=distro"
    exit 1
fi

DOCKER_FILE="$PWD/docker/Dockerfile.${AREA}"
echo "Building for @Home area: $AREA"

IMAGE_NAME="roborregos/home:$AREA-cpu"

if [ -n "$USE_CUDA" ]; then
    DOCKER_FILE="$PWD/docker/Dockerfile.${AREA}.cuda"
    IMAGE_NAME="roborregos/home:$AREA-cuda"
    echo "Using CUDA"
fi

if [ -n "$JETSON_L4T" ]; then
    DOCKER_FILE="$PWD/docker/Dockerfile.${AREA}.l4t-$JETSON_L4T"
    IMAGE_NAME="roborregos/home:$AREA-l4t-$JETSON_L4T"
    echo "Building for Nvidia Jetson with L4T: $JETSON_L4T"
fi

echo "Building docker image: $DOCKER_FILE"

docker build -t $IMAGE_NAME \
    -f $DOCKER_FILE $PWD