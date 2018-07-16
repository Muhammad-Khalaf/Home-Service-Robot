#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/walls.world --screen" &
sleep 10
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " & 
sleep 2
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch --screen" &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch " 
