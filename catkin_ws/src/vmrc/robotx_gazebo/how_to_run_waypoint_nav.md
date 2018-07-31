# run sand island gazebo
$ roslaunch robotx_gazebo sandisland_nctu.launch

# after gazebo is launched run the localization node in robotx_nctu
$ roslaunch localization gps_imu_localization_gazebo.launch
(note: some error will pop up, it doesn't matter)


# run WAMV PID controller node
rosrun robotx_gazebo WAMV_PID_controller.py


Services in the WAMV PID controller node:
# add waypoint(note: you should add at least one before you start and you can add waypoint(s) whenever you want to later)
rosservice call /add_waypoint 10 0 0 0

rosservice call /add_waypoint 10 10 0 0

rosservice call /add_waypoint -10 10 0 0


rosservice call /add_waypoint -10 -10 0 0

rosservice call /add_waypoint 10 -10 0 0

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

