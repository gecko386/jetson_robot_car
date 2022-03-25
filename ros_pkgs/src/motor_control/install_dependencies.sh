#!/bin/bash

sudo pip3 install -r requirements.txt

cd ..
git clone https://github.com/ros-drivers/ackermann_msgs.git
cd ackermann_msgs
git checkout ros2
cd ..
cd ..
colcon build --packages-select ackermann_msgs
