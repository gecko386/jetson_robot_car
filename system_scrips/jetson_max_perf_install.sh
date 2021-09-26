#!/bin/bash

source $(dirname $0)/utils.sh
information "#### install automatic fan management ####"
git clone https://github.com/Pyrestone/jetson-fan-ctl.git
cd jetson-fan-ctl
sudo ./install.sh
exit_if_last_command_failed "Cannot install automatic fan management. Please, check logs"

information "#### automatic fan management installed!!! ####"
