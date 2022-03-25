#!/bin/bash

source $(dirname $0)/utils.sh
information "#### configure access point ####"

information "#### set wifi card to a fixed interface ###"
sudo echo 'SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="b812", ATTR{type}=="1", NAME="wlan1"' >/etc/udev/rules.d/ap-card.rules

sudo apt install -y libgtk-3-dev build-essential gcc g++ pkg-config make hostapd git libqrencode-dev
exit_if_last_command_failed "Cannot install dependencies. Please, check logs"

cd access_point
git clone https://github.com/lakinduakash/linux-wifi-hotspot
cd linux-wifi-hotspot
make
sudo make install
exit_if_last_command_failed "Cannot install access point scripts. Please, check logs"
make clean

sudo cp ../access_point.service /etc/systemd/system/.
sudo cp ../create_ap.conf /etc/.

sudo systemctl enable access_point
sudo systemctl start access_point

exit_if_last_command_failed "Cannot install access point services. Please, check logs"

information "### Access point configured!!! ###"
exit 0
