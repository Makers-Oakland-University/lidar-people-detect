// Header file for the class
#include "PolyGen.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
  // Constructor with global and private node handle arguments
  PolyGen::PolyGen(ros::NodeHandle &n, ros::NodeHandle &pn)
  {
    // bind reconfigure server
    srv_.setCallback(boost::bind(&PolyGen::reconfig, this, _1, _2));

    poly_pub = n.advertise<geometry_msgs::PolygonStamped>("boundary_poly", 1);

    // publish the polygon every half second, there's no real need for this, we could also latch the message
    // but it opens the door later for us to make the polygon dynamic if we wanted to.
    timer = n.createTimer(ros::Duration(0.5), &PolyGen::publishPolygon, this);
  }

  void PolyGen::reconfig(PolyConfig &config, uint32_t level)
  {
    cfg_ = config;

    reconfig_points.clear();

    // I really really hate this, but it's the cleanest way I can think to do it since the reconfigure server won't allow
    // me to configure an array or array type object. I'm not typing all this myself, especially if someone decides they want a billion points or whatever.
    // here's the python code I generated the below code with:
    /*NUM_POINTS = 10

      for a in range(0, NUM_POINTS):
          print("//add point %d to list"%a)
          print("geometry_msgs::Point32 pt%d;"%a)
          print("pt%d.x = cfg_.poly_p%d_x;"%(a, a))
          print("pt%d.y = cfg_.poly_p%d_y;"%(a, a))
          print("reconfig_points.push_back(pt%d);"%a)
          print("")
          
    */

    // add point 0 to list
    geometry_msgs::Point32 pt0;
    pt0.x = cfg_.poly_p0_x;
    pt0.y = cfg_.poly_p0_y;
    reconfig_points.push_back(pt0);

    // add point 1 to list
    geometry_msgs::Point32 pt1;
    pt1.x = cfg_.poly_p1_x;
    pt1.y = cfg_.poly_p1_y;
    reconfig_points.push_back(pt1);

    // add point 2 to list
    geometry_msgs::Point32 pt2;
    pt2.x = cfg_.poly_p2_x;
    pt2.y = cfg_.poly_p2_y;
    reconfig_points.push_back(pt2);

    // add point 3 to list
    geometry_msgs::Point32 pt3;
    pt3.x = cfg_.poly_p3_x;
    pt3.y = cfg_.poly_p3_y;
    reconfig_points.push_back(pt3);

    // add point 4 to list
    geometry_msgs::Point32 pt4;
    pt4.x = cfg_.poly_p4_x;
    pt4.y = cfg_.poly_p4_y;
    reconfig_points.push_back(pt4);

    // add point 5 to list
    geometry_msgs::Point32 pt5;
    pt5.x = cfg_.poly_p5_x;
    pt5.y = cfg_.poly_p5_y;
    reconfig_points.push_back(pt5);

    // add point 6 to list
    geometry_msgs::Point32 pt6;
    pt6.x = cfg_.poly_p6_x;
    pt6.y = cfg_.poly_p6_y;
    reconfig_points.push_back(pt6);

    // add point 7 to list
    geometry_msgs::Point32 pt7;
    pt7.x = cfg_.poly_p7_x;
    pt7.y = cfg_.poly_p7_y;
    reconfig_points.push_back(pt7);

    // add point 8 to list
    geometry_msgs::Point32 pt8;
    pt8.x = cfg_.poly_p8_x;
    pt8.y = cfg_.poly_p8_y;
    reconfig_points.push_back(pt8);

    // add point 9 to list
    geometry_msgs::Point32 pt9;
    pt9.x = cfg_.poly_p9_x;
    pt9.y = cfg_.poly_p9_y;
    reconfig_points.push_back(pt9);

    ROS_INFO("Reconfigure Received for PolyGen %d points, polygon structure is:", cfg_.num_poly_points);
    for (int a = 0; a < cfg_.num_poly_points; a++)
    {
      ROS_INFO("   Point %d (%f, %f)", a, reconfig_points.at(a).x, reconfig_points.at(a).y);
    }
  }

  void PolyGen::publishPolygon(const ros::TimerEvent &event)
  {
    geometry_msgs::PolygonStamped poly_msg;

    if (reconfig_points.empty())
      return;

    // take only the points that are less than the num_poly_points, ignore all the others
    for (int a = 0; a < cfg_.num_poly_points; a++)
    {
      poly_msg.polygon.points.push_back(reconfig_points.at(a));
    }

    poly_msg.header.stamp = event.current_real;
    poly_msg.header.frame_id = "map";

    poly_pub.publish(poly_msg);
  }

}
