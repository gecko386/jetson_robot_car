#!/bin/bash

source $(dirname $0)/utils.sh

information "######## setting max performance on init ######"
if is_tegra; 
then

    sudo pip3 install jetson-stats
    exit_if_last_command_failed "Cannot install jetson stats toolkit. Please, check logs"

    sudo systemctl enable jetson_fan.service
    exit_if_last_command_failed "Cannot install jetson fan service. Please, check logs"
    sudo systemctl enable jetson_performance.service
    exit_if_last_command_failed "Cannot install jetson performance service. Please, check logs"
    
    sudo systemctl start jetson_fan.service
    exit_if_last_command_failed "Cannot start jetson fan service. Please, check logs"
    sudo systemctl start jetson_performance.service
    exit_if_last_command_failed "Cannot start jetson performance service. Please, check logs"
    
    
    information "all clocks set to max performance mode"
    warning "Please, check that unit has a good cooling system"
else
    warning "This script only works in tegra platform. Skip."
fi
exit 0
