#!/bin/bash

source $(dirname $0)/utils.sh

information "#### set default runlevel as non graphic ####"
sudo systemctl set-default multi-user.target 

exit_if_last_command_failed "Problems setting default runlevel. Please, check logs"
exit 0
