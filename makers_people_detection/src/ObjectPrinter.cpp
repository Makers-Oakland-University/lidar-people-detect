// Header file for the class
#include "ObjectPrinter.hpp"

// Namespace matches ROS package name
namespace makers_people_detect
{
    // Constructor with global and private node handle arguments
    ObjectPrinter::ObjectPrinter(ros::NodeHandle &n, ros::NodeHandle &pn) : listener_(buffer_)
    {
        ROS_INFO("Starting ObjectPrinter node");

        pn.param("file_path", file_path, std::string(""));
        ROS_INFO("Object printer will output object file to %s", file_path.c_str());

        marker_sub = n.subscribe("/viz", 1, &ObjectPrinter::marker_callback, this);
    }

    void ObjectPrinter::marker_callback(const visualization_msgs::MarkerArrayConstPtr &msg)
    {

        // this is far from efficent, now knowing that the marker array published by the kf tracker always has 6 elements I could remove the std::vectors
        // from this algorithm, but for now lets leave it, on-balance this isn't that much processing power being used.
        //  just take the incomming message and print it once a second so that we can show this is working properly.
        std::string output;
        std::vector<float> x_coords;
        std::vector<float> y_coords;

        std::vector<visualization_msgs::Marker> changed_markers;

        if (previous_markers.markers.size() == 0)
        {
            previous_markers = *msg;
            return;
        }

        if (!buffer_.canTransform("screen", "map", ros::Time(0)))
        {
            ROS_WARN("Waiting on transform screen -> map");
            return;
        }

        for (int a = 0; a < msg->markers.size(); a++)
            if (msg->markers[a].pose.position.x != previous_markers.markers[a].pose.position.x)
                changed_markers.push_back(msg->markers[a]);

        // get the size of the message
        int size = changed_markers.size();

        // resize things to fit.
        x_coords.resize(size);
        y_coords.resize(size);

        for (int a = 0; a < size; a++)
        {

            x_coords[a] = changed_markers[a].pose.position.x;
            y_coords[a] = changed_markers[a].pose.position.y;

            // take the point in the world frame and convert it to the screen frame, This will reference (0,0) to the upper left of the screen.
            geometry_msgs::TransformStamped world_transform;
            world_transform = buffer_.lookupTransform("screen", "map", ros::Time(0));
            geometry_msgs::Point p, p_out;
            p.x = changed_markers[a].pose.position.x;
            p.y = changed_markers[a].pose.position.y;
            tf2::doTransform<geometry_msgs::Point>(p, p_out, world_transform);

            // generate the output string;
            char part[100];
            sprintf(part, "%f,%f\n", p_out.x, p_out.y);

            output.append(part);
        }

        x_coords.shrink_to_fit();
        y_coords.shrink_to_fit();

        ROS_INFO_THROTTLE(1.0, "%s", output.c_str());

        // here is where we would append the data to a file.
        std::ofstream file;
        file.open(file_path + "/lidar_output.txt");
        file << output;
        file.close();

        previous_markers = *msg;
    }

}