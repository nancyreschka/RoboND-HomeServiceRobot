#!/bin/bash

cd /home/robond/workspace/catkin_ws; 
source /home/robond/workspace/catkin_ws/devel/setup.bash;
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/map/nancys_world.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/map/nancys_map.yaml

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
