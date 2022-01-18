#!/usr/bin/env bash

#Set the name of the docker image
CONTAINER_IMAGE="jetson/ros25"

#Set the name of the local directory you want to access within the image
LOCAL_DIRECTORY=~/Documents/github/RobotWars/src/robot/S1_Module/

#Give docker root user X11 permissions
sudo xhost +si:localuser:root

#enable SSH X11 forwaring inside container
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH


sudo docker run -i -t --privileged -v /dev/bus/usb:/dev/bus/usb -v /dev/input/:/dev/input/ --runtime nvidia --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    -v $LOCAL_DIRECTORY:/mnt \
    $CONTAINER_IMAGE
