// Header file for the class
#include "RealSenseCloudFilter.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
  // Constructor with global and private node handle arguments
  CloudFilter::CloudFilter(ros::NodeHandle &n, ros::NodeHandle &pn)
  {
    // subscriber for the depth points from the realsense camera
    scan_sub = n.subscribe("/camera1/depth/points", 1, &CloudFilter::cam_callback, this);
    poly_sub = n.subscribe("/boundary_poly", 1, &CloudFilter::poly_callback, this);

    // publisher for the final pointcloud
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  }

  // when we get the polygon we simply save it to a variable for use later.
  void CloudFilter::poly_callback(const geometry_msgs::PolygonStamped &msg)
  {
    saved_poly = msg;
  }

  // receive the depth map from the camera, convert to pointcloud, transform, and filter out points
  // we're not going to use, then publish the filtered cloud.
  void CloudFilter::cam_callback(const sensor_msgs::PointCloud2 &msg)
  {

    /* First step is to take the raw point cloud we receive from the camera 
    (converted externally to a pointcloud2) and convert it to a PointCloud, 
    this is a much easier datatype to work with for the later algorithm*/
    sensor_msgs::PointCloud raw_camera_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, raw_camera_cloud);


    /*The pointcloud is given in the frame of the camera, for everything that will come next
    we need to have the pointcloud in the world frame. We apply a TF transform in order to map
    the camera frame to the world frame.*/
    raw_camera_cloud.header.frame_id = msg.header.frame_id;
    sensor_msgs::PointCloud transformed_cloud;
    listener.transformPointCloud("/map", raw_camera_cloud, transformed_cloud);

    
    /*Now with the point cloud in the world frame we can perform the filtering operations on it. 
    We create a sensor message to hold the filtered points, then loop through every point in the
    input point cloud, for each point we will append it to the filtered cloud if
    it meets the following conditions: 
      - The points xy position is within the boundary polygon
      - The z position of the cloud is above the height threshold */
    int points = raw_camera_cloud.points.size();
    sensor_msgs::PointCloud transformed_filtered_cloud;

    // loop through every point, if it's inside the point cloud place it into filtered_cloud
    for (int a = 0; a < points; a++)
      if (inside_poly(transformed_cloud.points[a]) && transformed_cloud.points[a].z > Z_FILTER_HEIGHT)
        transformed_filtered_cloud.points.push_back(transformed_cloud.points[a]);

    //the filtered point cloud has no frame_id since it was created in a new message. 
    //we have to assign the frame_id to that of the transformed cloud.
    transformed_filtered_cloud.header.frame_id = transformed_cloud.header.frame_id;

    /*Finally take the filtered cloud and convert it to a pointcloud2, this is the message type
    that is used by the kf_tracker, then publish the result to the ROS system*/
    sensor_msgs::PointCloud2 filtered_cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(transformed_filtered_cloud, filtered_cloud2);
    cloud_pub.publish(filtered_cloud2);
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
