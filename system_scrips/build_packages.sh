#!/bin/bash

source $(dirname $0)/utils.sh

information "### build ros2 components inside a docker container ###"

sudo docker run -it --rm -v /home:/home -w /home/nvidia/jetson_robot_car/ros_pkgs base_image colcon build --symlink-install
exit_if_last_command_failed "Cannot build ros2 components. Please, check logs"

information "### software builded ###"