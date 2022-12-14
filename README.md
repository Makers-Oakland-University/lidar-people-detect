# Makers Pedestiran Detection

<!-- This repo contains the necessary code and information to create a reliable pedestrian tracking system for an indoor hallway.

The goal is to create a reliable method to detect people moving through the hallway outside the Augmented Reality Center, without relying on computer vision which is both difficult to develop and requires excessive processing power. 
Drivers, filtering, and other needed operations are all handled within this package. 

The code in this repo was developed for ROS Melodic, but should be capable of working on newer versions of ROS with modification to the setup.  -->

This repo contains the necessary code and information to create a reliable pedestrian tracking system for an indoor hallway. 
This repo is for experimenting with methods to detect people for the Augmented Reality Center interactive display project. 
The goal of this repo is to detect people moving through the hallway without utilizing conventional computer vision approaches which require significant processing horsepower. 
All systems here are built on ROS melodic (but should be runable in later versions of ros as well). 
The setup section provides all the information required to configure a PC running ROS to run these projects. 

# Games 

This system is designed to work together with the [ARC-Projector-Games](https://github.com/Makers-Oakland-University/ARC-Projector-Games) repo. If you're interested in creating games for the projector display then head there to check out the created games and the available examples. 

This system interfaces to the games through a textfile that is created and constantly written to by the ROS System contained in this repo. Creation of games does not require ROS knowledge or experience, the examples demonstrate how to read the positions from the file and use them to create interactive display projects. 

# Tracking Approaches

Currently two approaches have been tested, one uses an rplidar to detect pedestrians, the other utilizies the intel realsense camera. 
Each approach is documented in the below sections for use. 

[Lidar Demo Video (9/24/2022)](https://youtu.be/cwWT9cA8Y9Q)

[Informal Realsense Demo (9/30/2022)](https://www.youtube.com/watch?v=56GA1SOgyls)

## Lidar Approach

The lidar approach is built on ROS and makes use of the [RPLIDAR A1](https://www.adafruit.com/product/4010) by Slamtec. 
For this approach the lidar must be mounted at about waist height facing outwards, the lidar provides a 360 degree field of view of which only 180 degrees are utilized. 
The rplidar was tested and found to be capable of detecting objects through glass. 
Multiple lidars could also be implemented in this system to enable greater field of view if required. 

Data from the lidar is first received from the rplidar_ros driver. 
This data is converted from laser scan to PointCloud using the laser_geometry package. 
A bounding polygon, defined by PolyGen.cpp is then generated.
The polygon is then used in CloudFilter.cpp to remove any points that exist outside the polygon. 
The polygon should be configured such that it is within the bounds of the operational envirionment, and thereby remove any walls or other obstructions (polygon can be any shape). 
The pointcloud data within the polygon is then converted to a PointCloud2 and published to the multi_object_tracking_lidar node which handles the final detection of clusters and tracks people through the lidar. 
The ObjectPrinter node handles the printing of detections, this takes the marker output of the kf tracker and parses the xy position out of the marker array. 
In future iterations the ObjectPrinter will be used to pass this information to an external game or script.

This code also contains a camera driver, the purpose of this is simply to collect images of the environment to make sense of rosbags, it does not provide any information when operating normally. 

The lidar approach can be run with: 
```roslaunch makers_people_detection detect.launch```


## Intel Realsense Approach

The realsense approach is built on the [Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/). 
This camera provides range relative to itself which is used to create the depth map of the environment. 
Currently, this camera must be mounted directly above the area of interest facing directly down. 

The realsense camera is interfaced through the [ROS realsense2 camera node](http://wiki.ros.org/realsense2_camera). 
This enables the ROS system to obtain a depth map directly from the camera, the depth map itself is provided in the camera's reference frame. 
The depth cloud is then converted into a point cloud through the [depth_image_proc](http://wiki.ros.org/depth_image_proc) nodelet which converts the depth map into a point cloud in the camera's reference frame. 
A transform is then used in the realsenseCloudFilter node to convert the point cloud from the camera's reference fram to the world frame. 

With the point cloud in the world frame the extraction of relevant points is first performed by removing all points outside of the polygon generated by the PolyGen node. 
The PolyGen node should be configured for the final application in such a way that the walls of the environment are not included in the final point cloud, this enables later clustering algorithms to operate. 
With the point cloud filtered all points below a set height threshold (currently 1.5 meters for the makers lab) are removed from the cloud. 
The height should be configured to only include objects above waist height, this removes extraneous data that would interfere with clustering later. 
Finally, the filtered point cloud is published to the /filtered_cloud topic where it is read by the [multi-object-tracking-lidar](http://wiki.ros.org/multi_object_tracking_lidar) node which extracts objects from the cloud. 

As of 9/30/2022 this approach is incomplete, the object tracking functionality is not yet functional. 
Further work must be done to make this approach robust. 

The realsense approach can be run with: 
```roslaunch makers_people_detection realsense_detect.launch```


# Setup

Requires slamtec ROS driver https://github.com/Slamtec/rplidar_ros and multi-object-tracking-lidar package http://wiki.ros.org/multi_object_tracking_lidar


Will also require Hector Slam, libuvc, and laser geometry:

```sudo apt-get install ros-melodic-hector-slam ros-melodic-libuvc-camera ros-melodic-laser-geometry ros-melodic-realsense2-camera ros-melodic-depth-image-proc ros-melodic-rplidar-ros ros-melodic-multi-object-tracking-lidar ros-melodic-depth-image-proc```


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
To test from rosbag you will need to publish the /clock topic and only the raw image  + scan topics with the following line: 
```
rosbag play your_bag.bag --clock --loop --topics /scan /image_raw /clock
```
And toggle the use_sim_time environment variable 
```
rosparam set use_sim_time True
```


# ToDo 
- Clean up polygen so that it doesn't require manual re-entry whenever the environment is changed. 
