# lidar-people-detect

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

# Approach

Detection of people is performed by first definining a boundary in PolyGen.cpp, this polygon is used to filter out walls and other objects from the lidar point cloud.
From there the pointcloud is filtered...