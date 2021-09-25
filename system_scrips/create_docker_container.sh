#!/bin/bash

source $(dirname $0)/utils.sh


information "### Create docker container with ROS2 Galactic and pytorch ###"
git clone https://github.com/dusty-nv/jetson-containers.git

exit_if_last_command_failed "Cannot download utils. Please, check logs"
cd jetson-containers
./scripts/docker_build_ros.sh --distro galactic  --with-pytorch
exit_if_last_command_failed "Cannot create container. Please, check logs"

information "ROS2 Galactic container created"
