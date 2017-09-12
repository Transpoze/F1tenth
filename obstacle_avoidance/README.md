## How to use ZED
ZED is claimed to be the fastest depth camera in terms of framerate (1080p 30fps, WVGA 100fps) and it's the first sensor
to introduce indoor and outdoor long range depth perception (0.5-20m, 110deg max).
Here is the official [documentation](https://www.stereolabs.com/documentation/overview/getting-started/introduction.html).

### Installation on Jetson TK1
Please follow the instruction [here](https://www.stereolabs.com/blog/index.php/2015/09/24/getting-started-with-jetson-tk1-and-zed/).
Some important notes:
- You need a Linux host computer to install JetPack and setup TK1 for ZED
- Always try to use USB 3.0, even if the official documentation claims that USB 2.0 works
- If you have to flash TK1 (usually this is only for installing the OS on TK1), you can check [this video](https://devtalk.nvidia.com/default/topic/1001763/jetson-tx2/error-jetpack-must-be-run-on-x86_64-host-platform-detected-aarch64-platform-/) for reference.
- The [lastest version of ZED SDK for TK1](https://www.stereolabs.com/developers/release/1.2/) is 1.2 and it stops updating.
- TK1 only works with CUDA 6.5
- Default OpenCV installed by JetPack is 2.7
- You have to use c++ to develop applications with ZED

### Retrieving Data from ZED
#### With API
You can look into ZED API [here](https://www.stereolabs.com/developers/documentation/API/index.html). Interfaces are written in C++. This is more flexible for customizing your own project, and you can use it with [Point Cloud Library](http://pointclouds.org/) etc. to process point cloud data and images.
#### With ROS
Use catkin ROS package **zed_ros_wrapper**. It creates several topics publishing original image data, point cloud, etc. Check on [ros.org](http://wiki.ros.org/zed-ros-wrapper) and [Github](https://github.com/stereolabs/zed-ros-wrapper) for:
* Installation
* How to run/test the program
* Published topics

## Obstacle Avoidance
In this project (SML Summer job 26.6.2017-6.9.2017), a ROS package named **obstacle_detection** is created for the f1tenth vehichle to avoid single obstacle on its path. Any solid object will be regarded as obstacle without distinction. If several obstacles exists, they will be combined and considered as one large obstacle. In general, the package works as a ROS node which reads in point cloud data from ZED camera and sends commands, i.e. a new fixed waypoint, to the controller to bypass the obstacle if there is any. We accomplished such avoidance in two steps: detection and dynamic path planning. The main script is [obstacle_avoidance.py]. Please see details below.
### Obstacle Detection
Our node subscribes to the topic */zed/point_cloud/cloud_registered* to retrieve standard point cloud data in ros message type [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) from ZED. For the sake of computation cost, the point cloud is evenly sampled with a constant sampling step by calling python script *[readpoint.py]*. Also, a dynamic detection region ahead of the camera was set up to reduce not only computation time but also false detection rate. Any point outside this region will be discarded. The range, i.e. width and depth, of this region is changing dynamically based on the current x and y direction velocity of the vehicle with respect to the local coordinate system. This part is realized in the function *adjust_range()*.
Obstacle(s) is detected when the number of points in the detection region exceeds an empirically determined threshold called *point_count_min*. The threshold increases if you reduce the sampling step. Such detection logic works fine for all kinds of objects including human beings, boxes, small hills aside the road, etc. According to experiments, detection is not robust when the obstacle is within 0.5 meters and slightly suffers from the illumination. When the obstacle is too close, the dection result jumps between true and false. And, when the obstacle is very bright under the sun, it's more difficult to make correct detection result.
### Dynamic Path Planning
If the detection shows there is obstacle ahead, which essentially means that the number of points in the point cloud is too large to be a false detection, it triggers dynamic path planning.

## Troubleshooting
### ZED
**Problem:** Test ZED with ZED Explorer after installing everything, but it shows nothing, even though the camera can be recognized (you can tell by checking if it recognizes the serial number on the up-right corner).
**Solution:**
With reference to [this github issue](https://github.com/stereolabs/zed-ros-wrapper/issues/28)
   1. check if there is conf file in /usr/local/zed/settings, the filename should be SN\*\*\*\*.conf. If not, do:
/usr/local/zed/tools/ZED\ Explorer --dc # or --download_calibration
   2. check if the computer recognize the USB port:
lsusb | grep 2b03:f580 | wc -l
should return 1 if ZED is detected
   3. make sure ZED is connected by USB3.0, and change the usb_port_owner = 2 via:
sudo vi /boot/etlinux/extlinux.conf
   4. make sure Jetson TK1 is properly setup (JetPack, CUDA and ZED SDK), check out [this instruction](
https://www.stereolabs.com/blog/index.php/2015/09/24/getting-started-with-jetson-tk1-and-zed/)
