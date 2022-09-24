// ROS and node class header file
#include <ros/ros.h>
#include "PolyGen.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "PolyGen");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    makers_people_detect::PolyGen node(n, pn);

    // Spin and process callbacks
    ros::spin();
}