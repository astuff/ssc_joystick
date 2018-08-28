#!/bin/bash

wget https://s3.us-east-2.amazonaws.com/astuff-common-lib/ubuntu_1404/libas-common_2.1.0-0~14.040_amd64.deb -O /tmp/libas-common.deb
dpkg -i /tmp/libas-common.deb
apt-get install software-properties-common -y
add-apt-repository ppa:ubuntu-toolchain-r/test -y
apt-get update -qq
apt-get install gcc-5 g++-5 -y
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 1 --slave /usr/bin/g++ g++ /usr/bin/g++-5
