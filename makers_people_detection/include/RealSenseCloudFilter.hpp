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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>

#include <dynamic_reconfigure/server.h>
#include <makers_people_detection/PeopleDetectConfig.h>

// Adapted from the cloud_filter node. Since the realsense camera has specific requirements,
// I didn't want to destroy functionality that already works well with the lidar

// Namespace matches ROS package name
namespace makers_people_detect
{

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)
#define INSIDE 0
#define OUTSIDE 1

// Z filter height, all points below this height will be removed from the final pointcloud
// TODO move this to a launch parameter.
#define Z_FILTER_HEIGHT 1.3 // meters
#define Z_FILTER_HEIGHT_UPPER 1.8

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
        void reconfig(PeopleDetectConfig &config, uint32_t level);

        laser_geometry::LaserProjection projector;

        geometry_msgs::PolygonStamped saved_poly;
        ros::Subscriber scan_sub;
        ros::Subscriber poly_sub;
        ros::Publisher cloud_pub, marker_pub;
        tf::TransformListener listener;

        dynamic_reconfigure::Server<PeopleDetectConfig> srv_;
        PeopleDetectConfig cfg_;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // KD search tree object for use by PCL functions
        pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;
    };

}