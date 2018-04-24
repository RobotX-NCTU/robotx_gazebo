# RobotX Gazebo
## Introduction
This is the gazeo simulation of RobotX competition

## Prerequisites
1. Install dependencies 
the shellscript will install the gazebo plugins and python pkgs which we need.
```
$ source dependencies_common.sh
```
2. catkin_make
```
$ cd ~/robotx_gazebo/catkin_ws/
$ catkin_make
```
## Hardware and Model
- WAM-V 
  - Hardware
    - HDL32E LiDAR (not yet)
    - GPS
    - IMU
    - Front rgb camera * 2 (1280*720) 
    - dynamincs and thruster
  - Software
    - p3d (3D Position Interface for Ground Truth)
    - wind 
- Tasks Models
  - Ocean
  - sandisland
  - Green buoys 
  - Red buoys
  - Black buoys with A3, A5, A7
  - Dock (not real size)

## How to run the example
### Launch the world 
the world is about coarse approximation of the RobotX competition area, Sand Island, Honolulu, HI.
```
$ roslaunch robotx_gazebo sandisland_nctu.launch
```
### Control the WAM-V 
there are 3 ways that can control the WAM-V
1. rostopic pub
```
$ rostopic pub --once /cmd_drive robotx_gazebo/UsvDrive '{left: 0.5, right: -0.5}'
```
2. Keyboard
```
roslaunch robotx_gazebo usv_keydrive.launch
```
3. Joytick
``` 
robotx_gazebo usv_diffdrive.launch
```

## Remind
* Add sensors on robot </br>
please make the xacro file under vmrc/robotx_gazebo/urdf, and include the file in wamv_description/wamv_robotx_nctu.xacro </br>
* Build the worlds about task </br>
please save the file under vmrc/robotx_gazebo/worlds and chanege the arg "world" in vmrc/robotx_gazebo/sandisland_nctu.launch </br>
* Add models or 3D file such like buoys, *.dae ...... </br>
 please save the file under vmrc/robotx_gazebo/models
