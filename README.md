# RoboND-HomeServiceRobot
This is the repository for my Udacity Robotics Software Engineer Nanodegree Project - Home Service Robot. It contains a simulation world with Gazebo which includes a turtlebot robot. In this project I created a map from my simulated environment using SLAM, setup the navigation for my robot using AMCL and simulated virtual objects in RViz.

I implemented the following algorithm:
- a virtual object is displayed at the pickup zone
- the robot navigates to the pickup position
- the robot picks up the virtual object (object disappears, pause for 5 seconds)
- the robot travels to the drop off zone
- once the robot reaches the dropp off zone the virtual object is published at the drop off zone

### Output
The HomeServiceRobot ist launched by executing the home_service-script.
After launching the world the building, the obstacles, and the turtlebot are displayed inside a Gazebo World, RViz is launched as well as the ROS-nodes for navigating the robot and showing the virtual object. It should launch as follow:
![alt text](images/homeServiceRobot.gif)

### Directory Structure
```
    .RoboND-HomeServiceRobot                    # main folder 
    ├── add_markers                             # add_marker node 
    │   ├── src                                 # source folder for C++ scripts
    │   │   ├── add_markers.cpp
    │   ├── srv                                 # service folder for ROS services
    │   │   ├── PositionAction.srv
    │   ├── CMakeLists.txt                      # compiler instructions
    │   ├── package.xml                         # package info
    ├── images                                  # output images
    │   ├── homeServiceRobot.gif
    ├── map                                     # map files
    │   │   ├── nancys_map.gpm                  # contains a map file for the world
    │   │   ├── nancys_map.yaml                 # contains parameters for nancys_map.gpm
    │   │   ├── nancys_world.world
    ├── pick_objects                            # pick_objects C++ node
    │   ├── src                                 # source folder for C++ scripts
    │   │   ├── pick_objects.cpp
    │   ├── CMakeLists.txt                      # compiler instructions
    │   ├── package.xml                         # package info
    ├── rvizConfig                              # rviz configuration files
    │       ├── navigation.rviz
    ├── scripts                                 # shell script files
    │   ├── add_marker.sh
    │   ├── home_service.sh
    │   ├── launch.sh
    │   ├── pick_objects.sh
    │   ├── test_navigation.sh
    │   ├── test_slam.sh
    ├── slam_gmapping                           # submodule gmapping (gmapping_demo.launch file)
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                               # submodule turtlebot_teleop (keyboard_teleop.launch file)
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions                  # submodule turtlebot_rviz_launchers (view_navigation.launch file)
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator                     # submodule turtlebot_gazebo (turtlebot_world.launch file)
    │   ├── turtlebot_gazebo
    │   ├── ... 
    ├── .gitmodules                             # contains information about submodules
    └── README.md
```

### Steps to launch the simulation

#### 1 Clone the repository in the catkin workspace i.e. /home/workspace/catkin_ws/src
```sh
$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/nancyreschka/RoboND-HomeServiceRobot.git
```

#### 2 Initialize submodules
```sh
$ cd RoboND-HomeServiceRobot
$ git submodule init
$ git submodule update
```

#### 3 Compile the code
```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

#### 4 Launch the robot inside the Gazebo world and RViz
```sh
$ cd /home/workspace/catkin_ws/src/RoboND-HomeServiceRobot/scripts
$ ./home_service.sh
```

### The Shell Script Files

#### launch.sh

First script should launch Gazebo and Rviz in separate instances of terminals. xterm is used as a terminal.

#### test_slam.sh

The script deploys a turtlebot inside the environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz, to test if I can manually perform SLAM by teleoperating my robot. The goal of this step was to manually test SLAM.

#### test_navigation.sh

This script launches Gazebo, Rviz and AMCL for localization. The goal was to pick two different goals and test the robot's ability to reach them and orient itself with respect to them. The ROS Navigation stack is used, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan the robot trajectory from start to goal position. The ROS navigation stack permits the robot to avoid any obstacle on its path by re-planning a new trajectory once the robot encounters them.

#### pick_objects.sh

This script launches Gazebo, Rviz, AMCL for localization and the pick_objects node. The node is communicating with the ROS navigation stack and autonomously sending successive goals for the robot to reach. The ROS navigation stack creates a path for the robot based on Dijkstra's algorithm, a variant of the Uniform Cost Search algorithm, while avoiding obstacles on its path. The robot ist travelling to a pickup position and afterwards to a drop off position.

#### add_marker.sh

This script launches Gazebo, Rviz, AMCL for localization, the pick_objects node and the add_markers node. The add_markers node models a virtual object with markers in rviz. The virtual object is the one being picked and delivered by the robot, thus it first appears in its pickup zone, and then in its drop off zone once the robot reaches it.

#### home_service.sh

This script launches Gazebo, Rviz, AMCL for localization, the pick_objects node and the add_markers node. The pick_objects node and the add_markers node communicate while the robot is travelling to the pickup and drop off zone, so the markers appear and disappear depending on the robot's position. The communication is realized with a service server and client. Once the robot reaches the pickup zone the node pick_objects sends a request to hide the object to add_markers. after reaching the drop off zone the node pick_objects sends a request to show the object at the drop off zone to add_markers.
