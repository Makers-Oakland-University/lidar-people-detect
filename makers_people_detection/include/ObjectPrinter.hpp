// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*Object Printer

This class handles the reception and parsing of the incomming markers from the KF Tracker.
At the moment this node only outputs the result to the terminal. Future iterations will
make this node responsible for handling the transmission of information to the external game.

This will most likely be done through the use of a text file that both programs have access to.*/

// Namespace matches ROS package name
namespace makers_people_detect
{

  class ObjectPrinter
  {
  public:
    ObjectPrinter(ros::NodeHandle &n, ros::NodeHandle &pn);

  private:
    // Node-specific stuff here

    void marker_callback(const visualization_msgs::MarkerArrayConstPtr &msg);

    ros::Subscriber marker_sub;
    visualization_msgs::MarkerArray previous_markers;
    std::string file_path;

    tf2_ros::TransformListener listener_;
    tf2_ros::Buffer buffer_;
  };

}