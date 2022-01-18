#!/usr/bin/env bash

CONTAINER_IMAGE="jetson/ros25"

#Give docker root user X11 permissions
sudo xhost +si:localuser:root

#enable SSH X11 forwaring inside container
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH


sudo docker run -i -t --privileged -v /dev/input/:/dev/input/ --runtime nvidia --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    -v ~/Documents/ws:/mnt \
    $CONTAINER_IMAGE
