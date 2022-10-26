// Header file for the class
#include "RealSenseCloudFilter.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
  // Constructor with global and private node handle arguments
  CloudFilter::CloudFilter(ros::NodeHandle &n, ros::NodeHandle &pn) : tf_listener(tf_buffer), kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    // subscriber for the depth points from the realsense camera
    scan_sub = n.subscribe("/camera1/depth/points", 1, &CloudFilter::cam_callback, this);
    poly_sub = n.subscribe("/boundary_poly", 1, &CloudFilter::poly_callback, this);

    // bind reconfigure server
    srv_.setCallback(boost::bind(&CloudFilter::reconfig, this, _1, _2));

    // publisher for the final pointcloud
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("clustered_markers", 1);
  }

  // when we get the polygon we simply save it to a variable for use later.
  void CloudFilter::poly_callback(const geometry_msgs::PolygonStamped &msg)
  {
    saved_poly = msg;
  }

  void CloudFilter::reconfig(PeopleDetectConfig &config, uint32_t level)
  {
    cfg_ = config;
  }

  // receive the depth map from the camera, convert to pointcloud, transform, and filter out points
  // we're not going to use, then publish the filtered cloud.
  void CloudFilter::cam_callback(const sensor_msgs::PointCloud2 &msg)
  {
    // transform to map from camera frame
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("map", msg, transformed_msg, tf_buffer);

    // create a pcl cloud that can be processed
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_msg, *cloud);

    // passthrough filter based on the parameters of the dynamic reconfigure
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> filter;

    filter.setInputCloud(cloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(cfg_.x_min, cfg_.x_max);
    filter.filter(*indices);

    filter.setIndices(indices);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(cfg_.y_min, cfg_.y_max);
    filter.filter(*indices);

    filter.setIndices(indices);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(cfg_.z_min, cfg_.z_max);
    filter.filter(*cloud);

    // Run through a voxel grid filter to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud);
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
    downsample.filter(*cloud);

    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(cloud);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*cloud_normals);

    // Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    for (int i = 0; i < cloud_normals->points.size(); i++)
    {
      float c_ratio = pow(cloud_normals->points[i].normal_z, 2);

      if (c_ratio < cfg_.normals_filter_ratio)
        non_vertical_normals.indices.push_back(i);
    }
    pcl::copyPointCloud(*cloud, non_vertical_normals, *cloud);

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
      mark.color.b = 1.0; 
      mark.color.g = 1.0;

      pcl::PointXYZ min_point, max_point;
      pcl::getMinMax3D(*cluster, min_point, max_point);
      mark.pose.position.x = (min_point.x + max_point.x) / 2;
      mark.pose.position.y = (min_point.y + max_point.y) / 2;
      mark.pose.position.z = (min_point.z + max_point.z) / 2;

      mark.pose.orientation.w = 1.0; // everything else will default to 0.

      mark.type = visualization_msgs::Marker::SPHERE;

      mark.scale.x = max_point.x - min_point.x;
      mark.scale.y = max_point.y - min_point.y;
      mark.scale.z = max_point.z - min_point.z;

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

  // adapted from online resource https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
  int CloudFilter::inside_poly(geometry_msgs::Point32 p)
  {
    int counter = 0;
    int i;
    double xinters;
    geometry_msgs::Point32 p1, p2;
    int N = saved_poly.polygon.points.size();

    p1 = saved_poly.polygon.points[0];
    for (i = 1; i <= N; i++)
    {
      p2 = saved_poly.polygon.points[i % N];
      if (p.y > MIN(p1.y, p2.y))
      {
        if (p.y <= MAX(p1.y, p2.y))
        {
          if (p.x <= MAX(p1.x, p2.x))
          {
            if (p1.y != p2.y)
            {
              xinters = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
              if (p1.x == p2.x || p.x <= xinters)
                counter++;
            }
          }
        }
      }
      p1 = p2;
    }

    // had to reverse the return values of this to get the proper results
    if (counter % 2 == 0)
      return (INSIDE);
    else
      return (OUTSIDE);
  }
}
