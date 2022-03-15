#!/bin/bash

if [[  -f "/ros_entrypoint.sh" ]];
then
    sed -i 's/exec "$@"/source "\/home\/nvidia\/jetson_robot_car\/install\/local_setup.sh"\nexec "$@"/g' /ros_entrypoint.sh;
fi