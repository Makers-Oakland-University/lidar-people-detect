// Header file for the class
#include "RealSenseCloudFilter.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
  // Constructor with global and private node handle arguments
  CloudFilter::CloudFilter(ros::NodeHandle &n, ros::NodeHandle &pn)
  {
    scan_sub = n.subscribe("/camera1/depth/points", 1, &CloudFilter::cam_callback, this);
    poly_sub = n.subscribe("/boundary_poly", 1, &CloudFilter::poly_callback, this);

    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  }

  void CloudFilter::poly_callback(const geometry_msgs::PolygonStamped &msg)
  {
    saved_poly = msg;
  }

  // parse the points inside the boundary into a point cloud.
  void CloudFilter::cam_callback(const sensor_msgs::PointCloud2 &msg)
  {
    sensor_msgs::PointCloud raw_camera_cloud;

    sensor_msgs::convertPointCloud2ToPointCloud(msg, raw_camera_cloud);

    raw_camera_cloud.header.frame_id = msg.header.frame_id;

    tf::StampedTransform transform;
    listener.lookupTransform("/camera1_depth_optical_frame", "/map", ros::Time(0), transform);

    sensor_msgs::PointCloud transformed_cloud;
    sensor_msgs::PointCloud transformed_filtered_cloud;

    listener.transformPointCloud("/map", raw_camera_cloud, transformed_cloud);

    /// find number of points in cloud
    int points = raw_camera_cloud.points.size();

    // loop through every point, if it's inside the point cloud place it into filtered_cloud
    for (int a = 0; a < points; a++)
      if (inside_poly(transformed_cloud.points[a]) && transformed_cloud.points[a].z > Z_FILTER_HEIGHT)
        transformed_filtered_cloud.points.push_back(transformed_cloud.points[a]);

    // header so TF doesn't get all annoyed
    transformed_filtered_cloud.header.frame_id = transformed_cloud.header.frame_id;

    // convert to pointcloud2 (pointcloud was much easier to work with for the above algorithm)

    sensor_msgs::PointCloud2 filtered_cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(transformed_filtered_cloud, filtered_cloud2);

    // publish the filtered cloud
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
