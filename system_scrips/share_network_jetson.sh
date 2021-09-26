#!/bin/bash

source $(dirname $0)/utils.sh

information "### Enable server as gateway. ###"

#servidor enable internet
sudo sysctl net.ipv4.ip_forward=1
exit_if_last_command_failed "Cannot enable ip forwarding. Please, check logs"
#enp1s0 nombre interface internet
#enp2s0 nombre interface where forwarding the internet

sudo iptables -t nat -A POSTROUTING -o enp4s0 -j MASQUERADE
exit_if_last_command_failed "Cannot enable post routing. Please, check logs"

sudo iptables -A FORWARD -i enp4s0 -o usb1 -m state --state RELATED,ESTABLISHED -j ACCEPT
exit_if_last_command_failed "Cannot configure iptables connection forward. Please, check logs"

sudo iptables -A FORWARD -i usb1 -o enp4s0 -j ACCEPT 
exit_if_last_command_failed "Cannot configure iptables forward. Please, check logs"

information "### Server now acts as gateway. ###"
