#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/PositionAction.h"

enum PositionName {PICKUP = 1, HIDE, DROPOFF};
ros::Publisher marker_pub;
bool showMarker = true;

void printMarker(PositionName pos)
{
  if (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker typeto CUBE
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -2;
    marker.pose.position.y = 2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    // as soon as a subscriber is available publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        ROS_WARN_ONCE("ROS not working");
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // depending on the action to perform show or hide the marker at the specific position
    switch(pos)
    {
      case PICKUP:
        // show marker only once at startup
        showMarker = false;
        ROS_INFO("show marker pickup");
        // publish marker at pickup zone
        marker_pub.publish(marker);
        break;        
      case HIDE:
        ROS_INFO("hide marker");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        // pause 5 seconds
        ros::Duration(5.0).sleep();
        break;
      case DROPOFF:
        ROS_INFO("show marker dropoff");
        marker.pose.position.x = 4;
        marker.pose.position.y = -2;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        break;
      default:
        ROS_INFO("add_markers - Invalid request received");
        break;
    }
  }
}

// callback function for service server to handle position actions
bool handle_position_action(add_markers::PositionAction::Request& req,
                  add_markers::PositionAction::Response& res)
{
  if(req.action == 1){
    ROS_INFO("PositionAction for service received - action: pickup (%d)", (int)req.action);
    printMarker(PICKUP);
  }
  else if (req.action == 2){
    ROS_INFO("PositionAction for service received - action: hide (%d)", (int)req.action);
    printMarker(HIDE);
  }
  else if (req.action == 3){
    ROS_INFO("PositionAction for service received - action: dropoff (%d)", (int)req.action);
    printMarker(DROPOFF);
  }
  else
    ROS_INFO("PositionAction for service received - INVALID action:%d", (int)req.action);

  // Return a response message
  res.msg_feedback = "PositionAction: " + std::to_string(req.action);
  if (req.action == 1 || req.action == 2 || req.action == 3)
    res.msg_feedback += " performed.";
  else
    res.msg_feedback += " NOT performed.";
  ROS_INFO_STREAM(res.msg_feedback);
  return true;
}

int main( int argc, char** argv )
{
  // initialize ROS and create publisher on visualization_marker topic
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Define a add_markers service with a handle_position_action callback function
  ros::ServiceServer service = n.advertiseService("/add_markers/PositionAction", handle_position_action);

  // show marker at initial position
  if(showMarker){
    printMarker(PICKUP);
  }

  // Handle ROS communication events
  ros::spin();
}
