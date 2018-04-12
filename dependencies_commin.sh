#!/bin/bash
set -e

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

set -x


sudo apt install -y \
	ros-kinetic-twist-mux-* \
	ros-kinetic-geographic-* \
	libbullet-dev \
	ros-kinetic-diff-drive-controller \
	ros-kinetic-joint-* \
	ros-kinetic-velodyne-* \
	ros-kinetic-controller-manager \
	ros-kinetic-gazebo-ros-control \
        ros-kinetic-opencv-apps \
        ros-kinetic-jskeus \
        ros-kinetic-euslisp \
        ros-kinetic-nmea-* \
        ros-kinetic-serial \
        ros-kinetic-geometry2 \
        ros-kinetic-velodyne-* \
	ros-kinetic-hector-gazebo-plugins

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install mercurial cmake pkg-config python python-pip git gazebo7 libgazebo7-dev ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-xacro ros-kinetic-joy libeigen3-dev wget
pip install --upgrade pip
pip install packaging
pip install vcstools
sudo pip install appdirs
