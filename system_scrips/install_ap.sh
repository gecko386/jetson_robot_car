#!/bin/bash
sudo apt install -y libgtk-3-dev build-essential gcc g++ pkg-config make hostapd

cd access_point
git clone https://github.com/lakinduakash/linux-wifi-hotspot
cd linux-wifi-hotspot
make
sudo make install
make clean

sudo cp ../access_point.service /etc/systemd/system/.
sudo cp ../create_ap.conf /etc/.

sudo systemctl enable access_point
sudo systemctl start access_point
