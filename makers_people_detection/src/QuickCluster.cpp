// Header file for the class
#include "QuickCluster.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
    // Constructor with global and private node handle arguments
    QuickCluster::QuickCluster(ros::NodeHandle &n, ros::NodeHandle &pn) : tf_listener(tf_buffer), kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
    {
        ROS_INFO("Starting QuickCluster");
        // bind reconfigure server
        srv_.setCallback(boost::bind(&QuickCluster::reconfig, this, _1, _2));

        marker_pub = n.advertise<visualization_msgs::MarkerArray>("clustered_markers", 1);
        cloud_pub = n.advertise<sensor_msgs::PointCloud2>("post_processed_cloud", 1);

        // subscriber for the depth points from the realsense camera
        scan_sub = n.subscribe("/filtered_cloud", 1, &QuickCluster::cloud_callback, this);
    }

    void QuickCluster::cloud_callback(const sensor_msgs::PointCloud2 &msg)
    {

        // ROS_INFO("RUNNING CLOUD CALLBACK");

        sensor_msgs::PointCloud2 transformed_msg;
        pcl_ros::transformPointCloud("map", msg, transformed_msg, tf_buffer);

        // create a pcl cloud that can be processed
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_msg, *cloud);

        // Cluster the Points then recombine
        std::vector<pcl::PointIndices> group_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;

        ece.setClusterTolerance(cfg_.cluster_tolerance);
        ece.setMinClusterSize(cfg_.min_cluster_points);
        ece.setMaxClusterSize(cfg_.max_cluster_points);
        kd_tree_->setInputCloud(cloud);
        ece.setSearchMethod(kd_tree_);
        ece.setInputCloud(cloud);


        ece.extract(group_indices);

        // split the cluster into a list of lists of points
        for (auto indices : group_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, indices, *c);
            c->width = c->points.size();
            c->height = 1;
            c->is_dense = true;

            cluster_clouds.push_back(c);
        }

        // now we need to loop through the clusters and generate a marker array
        pcl::PointXYZ min_point, max_point;
        visualization_msgs::MarkerArray mArray;
        int marker_num = 0;
        for (auto &cluster : cluster_clouds)
        {
            pcl::getMinMax3D(*cluster, min_point, max_point);
            visualization_msgs::Marker mark;
            mark.header.frame_id = "map";
            mark.id = marker_num++;

            mark.color.a = 0.2;
            mark.color.r = 1.0;
            mark.color.b = 1.0;
            mark.color.g = 1.0;

            pcl::PointXYZ min_point, max_point;
            pcl::getMinMax3D(*cluster, min_point, max_point);
            mark.pose.position.x = (min_point.x + max_point.x) / 2;
            mark.pose.position.y = (min_point.y + max_point.y) / 2;
            mark.pose.position.z = (min_point.z + max_point.z) / 2;

            mark.pose.orientation.w = 1.0; // everything else will default to 0.

            mark.type = visualization_msgs::Marker::SPHERE;

            mark.scale.x = (max_point.x - min_point.x > 0.5) ? max_point.x - min_point.x : 0.5;
            mark.scale.y = (max_point.y - min_point.y > 0.5) ? max_point.y - min_point.y : 0.5;
            mark.scale.z = 0.5;

            mArray.markers.push_back(mark);
        }
        marker_pub.publish(mArray);

        // recombine
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto &c : cluster_clouds)
            merged->points.insert(merged->points.begin(), c->points.begin(), c->points.end());

        merged->width = merged->points.size();
        merged->height = 1;
        merged->is_dense = true;

        // convert pointcloud to ros message
        sensor_msgs::PointCloud2 downsampled_cloud_ros;
        pcl::toROSMsg(*merged, downsampled_cloud_ros);
        downsampled_cloud_ros.header = pcl_conversions::fromPCL(cloud->header);

        cloud_pub.publish(downsampled_cloud_ros);
    }

    void QuickCluster::reconfig(PeopleDetectConfig &config, uint32_t level)
    {
        cfg_ = config;
    }

}