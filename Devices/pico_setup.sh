#! /bin/bash
sudo bash -c 'echo "deb https://labs.picotech.com/debian/ picoscope main" >/etc/apt/sources.list.d/picoscope.list'
wget -O - https://labs.picotech.com/debian/dists/picoscope/Release.gpg.key | sudo apt-key add -
wget https://labs.picotech.com/debian/pool/main/libp/libpicohrdl/libpicohrdl_2.0.17-1r1441_armhf.deb
wget https://labs.picotech.com/debian/pool/main/libu/libusbtc08/libusbtc08_2.0.17-1r1441_armhf.deb
sudo apt install -q ./libpicohrdl_2.0.17-1r1441_armhf.deb ./libusbtc08_2.0.17-1r1441_armhf.deb -y
git clone https://github.com/picotech/picosdk-python-wrappers.git
cd picosdk-python-wrappers && sudo python3 setup.py install