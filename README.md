# SML_summerproject

A ROS-package developed during summer 2017 for localization and control of the F1Tenth car with NVIDIA Jetson TK1. 



## Localizaton 
### Sensors
There are currently two sensors being used for localization:
- Razor IMU 9DoF - accelerometer, gyro and magnetometer data avaliable as an [Imu message](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) on `/imu`
- Emlid Reach RTK kit - see [reach instructions](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_localization/reach_instructions.md) for more info

All sensors are handeled by the `sensor_helpers` package. To launch the sensors (together with`serial_node` for the Teensy board) use `roslaunch sensor_helpers launch_sensors.launch`. Note that you might need to adjust the port names in the launch file. 

### Sensor fusion
Sensor fusion is handeled by the `f1tenth_localization` package. Incomming measurements from the sensors are fused using an Extended Kalman Filter (EKF) implemented in the ROS-package [robot_localization](http://docs.ros.org/kinetic/api/robot_localization/html/index.html). You can run the filter using `roslaunch f1tenth_localization ekf_gps_imu.launch`.

The EKF calculations take place in an instance of `ekf_localization_node` that estimates the state of the car and publishes the information as an [odometry message](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) to `/odometry/filtered`. There is also an instance of `navsat_transform_node` that transforms latitude/longitude measurements from the Reach into local coordinates. 

#### Frames
- the position is given in the `odom` frame which has its origin at the starting location of the car with the positive x-axis pointing north
- the velocity is given in the `base_link` frame which is rigidly mounted on the base of the car (i.e. the x-component is always forward velocity etc)


The `odom->base_link` map is provided by the `ekf_localization_node` 

Internally, `navsat_transform_node` uses the [utm coordinate system](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) and a `utm->odom` transform is provided by the node. For converting between utm and latitude/lingitude you can use the python-package [utm](https://pypi.python.org/pypi/utm).


## Obstacle avoidance
### ZED Camera
[ZED](https://www.stereolabs.com/), a dual lens depth camera, is used for perception. It calculates depth information to create a point cloud. By using a ros package called [zed-ros-wrapper](http://wiki.ros.org/zed-ros-wrapper), the point cloud data will be published as a standard [sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) message.  
#### Prerequisites of Using ZED Camera
On a Linux PC:
- Ubuntu 16.04
- ZED SDK (lastest version)
- CUDA 8
- ROS Kinetic

On NVIDIA Jetson TK1:
- Ubuntu 12.04 or 14.04 64 bits (recommended for host computer to setup Jetson TK1)
- LinuxforTegra r21.4 (for Jetson)
- ZED SDK 1.2 (latest version for TK1)
- CUDA 6.5
- ROS Indigo
For more details, check [how to use ZED](https://github.com/KTH-SML/SML_summerproject/blob/master/obstacle_avoidance/README.md)

### Obstacle Detection

### Dynamic Path Planning
