#!/bin/bash

cd /home/robond/workspace/catkin_ws; 
source /home/robond/workspace/catkin_ws/devel/setup.bash;
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/map/nancys_world.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/map/nancys_map.yaml
export RVIZ_CONFIG=/home/robond/workspace/catkin_ws/src/RoboND-HomeServiceRobot/rvizConfig/navigation.rviz

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "rosrun rviz rviz -d $RVIZ_CONFIG" &
sleep 10
xterm -e "rosrun pick_objects pick_objects" &
xterm -e "rosrun add_markers add_markers" &
