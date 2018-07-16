#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/walls.world --screen" &
sleep 10
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/walls_map.yaml" & 
sleep 2
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch --screen" &
sleep 5
xterm  -e  " rosrun add_markers add_markers " &
sleep 1
xterm  -e  " rosrun pick_objects pick_objects "

