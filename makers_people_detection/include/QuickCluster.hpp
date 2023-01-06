// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <dynamic_reconfigure/server.h>
#include <makers_people_detection/PeopleDetectConfig.h>

// Namespace matches ROS package name
namespace makers_people_detect
{

    class QuickCluster
    {
    public:
        QuickCluster(ros::NodeHandle &n, ros::NodeHandle &pn);
        void cloud_callback(const sensor_msgs::PointCloud2 &msg);
        void reconfig(PeopleDetectConfig &config, uint32_t level);

    private:
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;

        ros::Subscriber scan_sub;
        ros::Publisher marker_pub, cloud_pub;

        dynamic_reconfigure::Server<PeopleDetectConfig> srv_;
        PeopleDetectConfig cfg_;
    };

}