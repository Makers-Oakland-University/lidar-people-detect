// ROS and node class header file
#include <ros/ros.h>
#include "QuickCluster.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "QuickCluster");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    makers_people_detect::QuickCluster node(n, pn);

    // Spin and process callbacks
    ros::spin();
}
