// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>



// Namespace matches ROS package name
namespace makers_people_detect
{

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)
#define INSIDE 0
#define OUTSIDE 1

#define Z_FILTER_HEIGHT 1.5 //meters


    /* Subscribes to the /boundary_poly topic and the laser scan topic.
      Includes only points that exist within the boundary of /boundary_poly and
      publishes data on the /filtered_point_cloud topic   */

    class CloudFilter
    {
    public:
        CloudFilter(ros::NodeHandle &n, ros::NodeHandle &pn);

    private:
        void poly_callback(const geometry_msgs::PolygonStamped &msg);
        void cam_callback(const sensor_msgs::PointCloud2 &msg);
        int inside_poly(geometry_msgs::Point32 p);

        laser_geometry::LaserProjection projector;

        geometry_msgs::PolygonStamped saved_poly;
        ros::Subscriber scan_sub;
        ros::Subscriber poly_sub;
        ros::Publisher cloud_pub;
        tf::TransformListener listener;
    };

}