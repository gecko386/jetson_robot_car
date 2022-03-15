#!/bin/bash

source $(dirname $0)/utils.sh
information "#### executing ROS2 Galactic container  ####"
xhost +
sudo docker run --privileged -it --rm --gpus all --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp/argus_socket:/tmp/argus_socket -v /etc/udev/rules.d:/etc/udev/rules.d -v /dev:/dev -v /sys/class/gpio:/sys/class/gpio -v /sys/devices:/sys/devices -v /dev/gpiochip0:/dev/gpiochip0 -v /dev/gpiochip1:/dev/gpiochip1 -v /sys/firmware/devicetree/base:/sys/firmware/devicetree/base -e DISPLAY=$DISPLAY -w $HOME --mount src=/home,target=/home,type=bind base_image
