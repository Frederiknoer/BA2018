#!/bin/sh

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install git -y

sudo git clone https://github.com/Kitware/CMake.git ~/Libraries
cd ~/Libraries/CMake
git checkout v3.9.0-rc6
sudo ./bootstrap &&  sudo make && sudo make install

sudo apt-get install libssl-dev -y
git clone https://github.com/IntelRealSense/librealsense.git ~/Libraries
cd  ~/librealsense
git checkout v2.10.0
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-xenial.sh
sudo apt-get install libusb-1.0-0-dev pkg-config libgtk-3-dev -y
mkdir build && cd build
cmake ..
sudo make uninstall && make clean && make && sudo make install

