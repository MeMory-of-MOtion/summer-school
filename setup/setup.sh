#!/bin/bash

set -ex

export DEBIAN_FRONTEND=noninteractive

# Setup robotpkg
apt update -qqy
apt install -qqy gnupg2
apt-key add robotpkg.key
echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg' > /etc/apt/sources.list.d/robotpkg.list

# Install dependencies
apt update -qqy
apt install -qqy git python3-{matplotlib,pip} robotpkg-py36-{crocoddyl,tsid,qt5-gepetto-viewer-corba}
pip3 install --no-cache-dir jupyter meshcat

# Configure paths
echo /opt/openrobots/lib/python3.6/site-packages > /usr/lib/python3/dist-packages/robotpkg.pth
echo "export PATH=/opt/openrobots/bin:$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/opt/openrobots/lib" >> ~/.bashrc

# Cleanup (avoid useless space on docker image)
rm -rf /var/lib/apt/lists/*
