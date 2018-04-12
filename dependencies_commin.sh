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
