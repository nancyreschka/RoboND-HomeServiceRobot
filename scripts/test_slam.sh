#!/bin/bash

cd /home/robond/workspace/catkin_ws; 
source /home/robond/workspace/catkin_ws/devel/setup.bash;
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/map/nancys_world.world

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
