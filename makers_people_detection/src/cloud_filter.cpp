// ROS and node class header file
#include <ros/ros.h>
#include "CloudFilter.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "CloudFilter");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    makers_people_detect::CloudFilter node(n, pn);

    // Spin and process callbacks
    ros::spin();
}
