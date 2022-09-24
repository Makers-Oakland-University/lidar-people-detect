// Header file for the class
#include "PolyGen.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
  // Constructor with global and private node handle arguments
  PolyGen::PolyGen(ros::NodeHandle &n, ros::NodeHandle &pn)
  {
    // publish the polygon every half second, there's no real need for this, we could also latch the message
    // but it opens the door later for us to make the polygon dynamic if we wanted to.
    timer = n.createTimer(ros::Duration(0.5), &PolyGen::publishPolygon, this);

    poly_pub = n.advertise<geometry_msgs::PolygonStamped>("boundary_poly", 1);
  }

  void PolyGen::publishPolygon(const ros::TimerEvent &event)
  {
    geometry_msgs::PolygonStamped poly_msg;

    // now add the points
    geometry_msgs::Point32 pt1;
    pt1.x = 0;
    pt1.y = 0;
    pt1.z = 0;
    poly_msg.polygon.points.push_back(pt1);

    geometry_msgs::Point32 pt2;
    pt2.x = 1.89;
    pt2.y = 1.65;
    pt2.z = 0;
    poly_msg.polygon.points.push_back(pt2);

    geometry_msgs::Point32 pt3;
    pt3.x = 3.58;
    pt3.y = -1.59;
    pt3.z = 0;
    poly_msg.polygon.points.push_back(pt3);

    geometry_msgs::Point32 pt4;
    pt4.x = 1.62;
    pt4.y = -2.43;
    pt4.z = 0;
    poly_msg.polygon.points.push_back(pt4);

    poly_msg.header.stamp = event.current_real;
    poly_msg.header.frame_id = "map";

    poly_pub.publish(poly_msg);
  }

}