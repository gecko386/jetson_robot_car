#!/bin/bash

source $(dirname $0)/utils.sh
information "#### executing ROS2 Galactic container  ####"
sudo docker run -it --rm --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -w $HOME --mount src=/home,target=/home,type=bind ros:galactic-pytorch-l4t-r32.6.1 
