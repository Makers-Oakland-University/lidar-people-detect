// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <dynamic_reconfigure/server.h>
#include <makers_people_detection/PolyConfig.h>

// Namespace matches ROS package name
namespace makers_people_detect
{

  class PolyGen
  {
  public:
    PolyGen(ros::NodeHandle &n, ros::NodeHandle &pn);
    void reconfig(PolyConfig &config, uint32_t level);

  private:
    // Node-specific stuff here
    void publishPolygon(const ros::TimerEvent &event);

    // publishers and timers
    ros::Publisher poly_pub;
    ros::Timer timer;

    dynamic_reconfigure::Server<PolyConfig> srv_;
    PolyConfig cfg_;

    std::vector<geometry_msgs::Point32> reconfig_points;
  };

}