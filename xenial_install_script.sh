#!/bin/bash

wget https://s3.us-east-2.amazonaws.com/astuff-common-lib/ubuntu_1604/libas-common_2.0.0-0~16.040_amd64.deb -O /tmp/libas-common.deb
dpkg -i /tmp/libas-common.deb
