#!/bin/bash

source $(dirname $0)/utils.sh

go_to_folder_or_die /home/nvidia/jetson_robot_car/docker_builds/base_image
cp /proc/device-tree/compatible compatible
sudo docker build --rm -t "base_image" --build-arg BASE_IMAGE="dustynv/ros:galactic-pytorch-l4t-r32.7.1" -f Dockerfile .

exit_if_last_command_failed "Cannot create container. Please, check logs"

information "ROS2 Galactic container created"
