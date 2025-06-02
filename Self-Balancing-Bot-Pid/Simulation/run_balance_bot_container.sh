#!/bin/bash

IMAGE_NAME=balance-bot-sim-env
CONTAINER_NAME=balance-bot-sim-container
WORKSPACE_DIR=$(pwd)/ros2_ws

# setting DISPLAY manually (for X11 forwarding)
export DISPLAY=$(ip route | awk '/default/ {print $3}'):0
xhost +local:docker

# Kill old container if running
docker rm -f $CONTAINER_NAME 2>/dev/null

# Run container
docker run -it \
    --gpus all \
    --name $CONTAINER_NAME \
    --rm \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $WORKSPACE_DIR:/home/dev/ros2_ws \
    $IMAGE_NAME
