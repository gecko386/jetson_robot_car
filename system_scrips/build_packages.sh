#!/bin/bash

source $(dirname $0)/utils.sh

information "### build ros2 components inside a docker container ###"


git clone -b ros2 https://github.com/ros-drivers/ackermann_msgs.git /home/nvidia/jetson_robot_car/ros_pkgs/src/ackermann_msgs

sudo rm -fr /home/nvidia/jetson_robot_car/ros_pkgs/build
sudo rm -fr /home/nvidia/jetson_robot_car/ros_pkgs/log
sudo rm -fr /home/nvidia/jetson_robot_car/ros_pkgs/install

sudo docker run -it --rm -v /home:/home -w /home/nvidia/jetson_robot_car/ros_pkgs base_image colcon build --symlink-install
exit_if_last_command_failed "Cannot build ros2 components. Please, check logs"

information "### software builded ###"