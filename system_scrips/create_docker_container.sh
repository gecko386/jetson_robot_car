#!/bin/bash

source $(dirname $0)/utils.sh

information "### create docker containers ###"

information "### configure docker to use nvidia-runtime by default ###"
sudo cp docker-config/daemon.json /etc/docker/.
sudo systemctl restart docker.service
exit_if_last_command_failed "Cannot configure docker. Please, check logs"

#information "### Create docker container with ROS2 Galactic and pytorch ###"
#git clone https://github.com/dusty-nv/jetson-containers.git

#exit_if_last_command_failed "Cannot download utils. Please, check logs"
#cd jetson-containers
#./scripts/docker_build_ros.sh --distro galactic  --with-pytorch
#exit_if_last_command_failed "Cannot create container. Please, check logs"

go_to_folder_or_die /home/nvidia/jetson_robot_car/docker_builds/base_image
cp /proc/device-tree/compatible compatible
sudo docker build --rm -t "base_image" --build-arg BASE_IMAGE="dustynv/ros:galactic-pytorch-l4t-r32.6.1" -f Dockerfile .

exit_if_last_command_failed "Cannot create container. Please, check logs"

information "ROS2 Galactic container created"
