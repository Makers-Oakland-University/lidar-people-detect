// Header file for the class
#include "ObjectPrinter.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
    // Constructor with global and private node handle arguments
    ObjectPrinter::ObjectPrinter(ros::NodeHandle &n, ros::NodeHandle &pn)
    {
        ROS_INFO("Starting ObjectPrinter node");

        marker_sub = n.subscribe("/viz", 1, &ObjectPrinter::marker_callback, this);
    }

    void ObjectPrinter::marker_callback(const visualization_msgs::MarkerArrayConstPtr &msg)
    {

        //this is far from efficent, now knowing that the marker array published by the kf tracker always has 6 elements I could remove the std::vectors
        //from this algorithm, but for now lets leave it, on-balance this isn't that much processing power being used.

        // just take the incomming message and print it once a second so that we can show this is working properly.
        std::string output;
        std::vector<float> x_coords;
        std::vector<float> y_coords;

        // get the size of the message
        int size = msg->markers.size();

        // resize things to fit.
        x_coords.resize(size);
        y_coords.resize(size);

        for (int a = 0; a < size; a++)
        {

            x_coords[a] = msg->markers[a].pose.position.x;
            y_coords[a] = msg->markers[a].pose.position.y;

            // generate the output string;
            char part[100];
            sprintf(part, "%f,%f\n", msg->markers[a].pose.position.x, msg->markers[a].pose.position.y);

            output.append(part);
        }

        x_coords.shrink_to_fit();
        y_coords.shrink_to_fit();

        ROS_INFO_THROTTLE(1.0, "%s", output.c_str());

        //here is where we would append the data to a file.

    }

}