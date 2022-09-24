# lidar-people-detect

Requires slamtec ROS driver https://github.com/Slamtec/rplidar_ros


Will also require Hector Slam and libuvc:

```sudo apt-get install ros-melodic-hector-slam ros-melodic-libuvc-camera```


## Opening camera

if you run into issues with the camera and get the following message: 
```[ERROR] [1663976819.358003229]: Permission denied opening /dev/bus/usb/001/010
[ERROR] [1663976819.371416741]: Unable to open camera.```
then you'll need to run the following command:
```sudo chmod o+w /dev/bus/usb/001/003```

Follow this tutorial for more information on setting up the cameras https://msadowski.github.io/ros-web-tutorial-pt2-cameras/
