#!/bin/bash

sudo bash -c 'echo "deb https://labs.picotech.com/debian/ picoscope main" >/etc/apt/sources.list.d/picoscope.list'
wget -O - https://labs.picotech.com/debian/dists/picoscope/Release.gpg.key | sudo apt-key add -
wget https://labs.picotech.com/debian/pool/main/libp/libpicohrdl/libpicohrdl_2.0.17-1r1441_armhf.deb
wget https://labs.picotech.com/debian/pool/main/libu/libusbtc08/libusbtc08_2.0.17-1r1441_armhf.deb
sudo apt install --quiet ./libpicohrdl_2.0.17-1r1441_armhf.deb ./libusbtc08_2.0.17-1r1441_armhf.deb -y
sudo rm -rf libpicohrdl* libusbtc08*
sudo apt install git screen tree net-tools python3 python3-numpy python3-matplotlib python3-serial python3-pandas python3-pip -y

sudo pip3 install adafruit-circuitpython-vl6180x
git clone https://github.com/picotech/picosdk-python-wrappers.git
cd picosdk-python-wrappers && sudo python3 setup.py install
cd ~
echo ' '
echo 'Please install rpiDevices package using "git clone git@github.com:jse-springboard/rpiDevices.git".'
echo 'Attempting auto install now...'
echo ''
git clone git@github.com:jse-springboard/rpiDevices.git && cd $(python3 -c 'import picosdk; print(picosdk.__path__[0])') && cd ../../dist-packages/ && sudo mv ~/rpiDevices/ ./ && echo 'rpiDevices install SUCCESS!'
echo '------------------'
cd ~