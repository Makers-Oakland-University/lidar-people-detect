// ROS and node class header file
#include <ros/ros.h>
#include "ObjectPrinter.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "ObjectPrinter");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    makers_people_detect::ObjectPrinter node(n, pn);

    // Spin and process callbacks
    ros::spin();
}