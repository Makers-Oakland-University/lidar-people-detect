# lidar-people-detect

This repo contains the necessary code and information to create a reliable pedestrian tracking system for an indoor hallway.
The system is built on ROS and makes use of the [RPLIDAR A1](https://www.adafruit.com/product/4010) by Slamtec. 
The goal is to create a reliable method to detect people moving through the hallway outside the Augmented Reality Center, without relying on computer vision which is both difficult to develop and requires excessive processing power. 
Drivers, filtering, and other needed operations are all handled within this package. 

The code in this repo was developed for ROS Melodic, but should be capable of working on newer versions of ROS with modification to the setup. 

[Demo Video (9/24/2022)](https://youtu.be/cwWT9cA8Y9Q)


## Approach

Data from the lidar is first received from the rplidar_ros driver. 
This data is converted from laser scan to PointCloud using the laser_geometry package. 
A bounding polygon, defined by PolyGen.cpp is then generated.
The polygon is then used in CloudFilter.cpp to remove any points that exist outside the polygon. 
The polygon should be configured such that it is within the bounds of the operational envirionment, and thereby remove any walls or other obstructions (polygon can be any shape). 
The pointcloud data within the polygon is then converted to a PointCloud2 and published to the multi_object_tracking_lidar node which handles the final detection of clusters and tracks people through the lidar. 

This code also contains a camera driver, the purpose of this is simply to collect images of the environment to make sense of rosbags, it does not provide any information when operating normally. 

# Setup

Requires slamtec ROS driver https://github.com/Slamtec/rplidar_ros and multi-object-tracking-lidar package http://wiki.ros.org/multi_object_tracking_lidar


Will also require Hector Slam, libuvc, and laser geometry:

```sudo apt-get install ros-melodic-hector-slam ros-melodic-libuvc-camera ros-melodic-laser-geometry```


## Opening camera

if you run into issues with the camera and get the following message: 
```
[ERROR] [1663976819.358003229]: Permission denied opening /dev/bus/usb/001/010
[ERROR] [1663976819.371416741]: Unable to open camera.
```
then you'll need to run the following command:
```
sudo chmod o+w /dev/bus/usb/001/003
```

Follow this tutorial for more information on setting up the cameras https://msadowski.github.io/ros-web-tutorial-pt2-cameras/

## Testing from rosbag
To test from rosbag you will need to publish the /clock topic with the following statement: 
```
rosbag play --clock your_bag.bag
```
And toggle the use_sim_time environment variable 
```
rosparam set use_sim_time True
```


# ToDo 
Interface with external scripts to enable the parsed positions from the lidar to be used with a unity/pygame to display on the projector. 
