#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/PositionAction.h>

// Define a global client that can request services
ros::ServiceClient client;

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// define constants for actions
const int PICKUP = 1;
const int HIDE = 2;
const int DROPOFF = 3;

// call client to request a position action
void requestAction(int action){
   add_markers::PositionAction srv;
   srv.request.action = action;
   ROS_INFO("Request action: %d", srv.request.action);

   // Call the position action service and pass the requested action
   if (!client.call(srv))
      ROS_ERROR("Failed to call service handle_position_action");

   ROS_INFO("Answer: %s", srv.response.msg_feedback.c_str());
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Define a client service capable of requesting services from add_markers
  client = n.serviceClient<add_markers::PositionAction>("/add_markers/PositionAction");
 
  if (ros::ok())
  {
    // initialize a MoveBaseClient  
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // generate the goal where the robot should pickup an object
    move_base_msgs::MoveBaseGoal pickup_goal;

    // set up the frame parameters
    pickup_goal.target_pose.header.frame_id = "map";
    pickup_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup_goal.target_pose.pose.position.x = -2.0;
    pickup_goal.target_pose.pose.position.y = 2.0;
    pickup_goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Robot is traveling to the pick up zone");
    ac.sendGoal(pickup_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robot picked up the virtual object");
      // request hiding the virtual object
      requestAction(HIDE);

      // Wait 5 sec
      ros::Duration(5.0).sleep();

      // generate the goal where the robot should drop off the object
      move_base_msgs::MoveBaseGoal dropoff_goal;

      // set up the frame parameters
      dropoff_goal.target_pose.header.frame_id = "map";
      dropoff_goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      dropoff_goal.target_pose.pose.position.x = 4.0;
      dropoff_goal.target_pose.pose.position.y = -2.0;
      dropoff_goal.target_pose.pose.orientation.w = 1.0;

      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Robot is traveling to the drop off zone");
      ac.sendGoal(dropoff_goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Robot dropped off the virtual object");
        // request dropping off the virtual object
        requestAction(DROPOFF);
      }
      else
        ROS_INFO("Robot failed to drop off the virtual object for some reason");
    }
    else
      ROS_INFO("Robot failed to pick up the virtual object for some reason");
  }

  return 0;
}
