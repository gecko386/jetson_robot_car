#!/bin/bash

source $(dirname $0)/utils.sh

information "### add user to docker group"
sudo usermod -aG docker $USER
exit_if_last_command_failed "Cannot add user to docker group. Please, check logs"

information "### configure docker to use nvidia-runtime by default ###"
sudo cp docker-config/daemon.json /etc/docker/.
sudo systemctl restart docker.service
exit_if_last_command_failed "Cannot configure docker. Please, check logs"

information "### install additional packages ###"
sudo pip3 install docker-compose
exit_if_last_command_failed "Cannot install additional packages. Please, check logs"
information "docker environment configured"