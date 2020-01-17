#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = 1.0;
  pickup_goal.target_pose.pose.position.y = 1.0;
  pickup_goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the pick up zone");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot picked up the virtual object");

    // Wait 5 sec for move_base action server to come up
    ros::Duration(5.0).sleep();

    move_base_msgs::MoveBaseGoal dropoff_goal;

    // set up the frame parameters
    dropoff_goal.target_pose.header.frame_id = "map";
    dropoff_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    dropoff_goal.target_pose.pose.position.x = -2.0;
    dropoff_goal.target_pose.pose.position.y = -2.0;
    dropoff_goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Robot is traveling to the drop off zone");
    ac.sendGoal(dropoff_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Robot dropped off the virtual object");
    else
      ROS_INFO("Robot failed to drop off the virtual object for some reason");
  }
  else
    ROS_INFO("Robot failed to pick up the virtual object for some reason");

  return 0;
}
