#!/bin/bash

source $(dirname $0)/utils.sh
information "#### executing ROS2 Galactic container  ####"
xhost +
sudo docker run -it --rm --gpus all --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp/argus_socket:/tmp/argus_socket -e DISPLAY=$DISPLAY -w $HOME --mount src=/home,target=/home,type=bind base_image
