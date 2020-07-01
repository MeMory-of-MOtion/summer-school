#!/bin/bash

set -ex

export DEBIAN_FRONTEND=noninteractive

# Setup robotpkg
apt-get update -qqy
apt-get install -qqy gnupg2
apt-key add robotpkg.key
echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg' > /etc/apt/sources.list.d/robotpkg.list

# Install dependencies
apt-get update -qqy
apt-get install -qqy \
    python3-matplotlib \
    python3-pip \
    robotpkg-py36-crocoddyl \
    robotpkg-py36-tsid
pip3 install --no-cache-dir \
    jupyter \
    meshcat

# Configure paths
echo /opt/openrobots/lib/python3.6/site-packages > /usr/lib/python3/dist-packages/robotpkg.pth
echo "PATH=/opt/openrobots/bin:$PATH" > /etc/environment
echo "LD_LIBRARY_PATH=/opt/openrobots/lib" >> /etc/environment
echo "ROS_PACKAGE_PATH=/opt/openrobots/share" >> /etc/environment

# Cleanup (avoid useless space on docker image)
rm -rf /var/lib/apt/lists/*
