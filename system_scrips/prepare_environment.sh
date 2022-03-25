#!/bin/bash

source $(dirname $0)/utils.sh

information "#### prepare system ####"

./jetson_max_perf_install.sh

information "install new packages"
sudo apt update && sudo apt install -y python3-pip
sudo pip3 install --upgrade pip

./install_ap.sh
./install_ydlidar.sh

./configure_docker_environment.sh