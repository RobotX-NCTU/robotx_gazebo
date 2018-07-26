# run sand island gazebo
$ roslaunch robotx_gazebo sandisland_nctu.launch

# after gazebo is launched run the localization node in robotx_nctu
$ rosrun localization vel_remap_gazebo.py
$ roslaunch localization gps_imu_localization.launch
(note: some error will pop up, it doesn't matter)


# run WAMV PID controller node
rosrun robotx_gazebo WAMV_PID_controller.py


Services in the WAMV PID controller node:
# add waypoint(note: you should add at least one before you start and you can add waypoint(s) whenever you want to later)
rosservice call /add_waypoint "waypointx: 10.0
waypointy: 0.0
yaw: 0.0
vel: 0.0" 

rosservice call /add_waypoint "waypointx: 10.0
waypointy: 10.0
yaw: 0.0
vel: 0.0" 

rosservice call /add_waypoint "waypointx: -10.0
waypointy: 10.0
yaw: 0.0
vel: 0.0" 


rosservice call /add_waypoint "waypointx: -10.0
waypointy: -10.0
yaw: 0.0
vel: 0.0" 

rosservice call /add_waypoint "waypointx: 10.0
waypointy: -10.0
yaw: 0.0
vel: 0.0" 

# start
rosservice call /start_waypoint_nav "{}"

# pause
rosservice call /pause_waypoint_nav "{}"

# clear all waypoints
rosservice call /clear_waypoints "{}"

# station keep on target waypoint
rosservice call /station_keep "{}"

# unlock station keep on target waypoint
rosservice call /station_keep_unlock "{}"

