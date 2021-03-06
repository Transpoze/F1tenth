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

For more details on ZED, check [how to use ZED](https://github.com/KTH-SML/SML_summerproject/tree/master/f1tenth_obstacle_avoidance)

### Obstacle Detection and Avoidance
In general, obstacle detection and avoidance are achieved by analyzing point cloud within a certain detection region generated by ZED camera. A large enough point cloud indicates existance of obstacle(s). We estimate the distance and edge of the obstacle(s) so that the vehicle could take an appropriate route to bypass the obstacle(s). This algorithm requires localization of the vehicle and a motion planner, also it involves many emprical parameters which need to be tuned. In the end of the summer project, the car manages to avoid single static obstacle with width round 0.5 meters at a speed around 1-1.5m/s. Larger obstacle hasn't been tested yet. The main script *[obstacle_avoidance.py](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_obstacle_avoidance/obstacle_detection/src/obstacle_avoidance.py)* runs in at least 10Hz.

[Here is the detailed description on algorithm, useful notes and troubleshooting.](https://github.com/KTH-SML/SML_summerproject/tree/master/f1tenth_obstacle_avoidance)

## Control System
### Setting up Motors with Teensy

The car used in the project is [F1Tenth](http://f1tenth.org/car-assembly). The Traxxas ESC is the power electronic controller that takes in control signals from the [Teensy](https://www.pjrc.com/store/teensy32.html) and delivers power to the motors. The Elecronic Speed Controller is equipped with Low-Voltage protection, which needs to be disabled for the car to run. Check how to program the [ESC](https://traxxas.com/support/Programming-Your-Traxxas-Electronic-Speed-Control) to DISABLE the low voltage protection while using NiMH batteries. The Teensy is already programmed to provide PWM signals(serially) to the ESC. If it is required to alter the code inside Teensy,use [Arduino IDE](https://www.arduino.cc/en/Main/OldSoftwareReleases) along with the [software plugin](https://www.pjrc.com/teensy/td_download.html). The teensy needs to be connected to the Jetson via USB 2.0/3.0 port.

### Pure-Pursuit Controller & PID controller:

The `control1` Package handles the controllers for the car. The Pure-pursuit controller Looks ahead a certain distance in meters and calculates a curvature to get to that point. The main parameter that decides the performance is the Look ahead distance. The controller takes in position and velocity commands from [odometry message](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) `/odometry/filtered` and Waypoints from HMI in the form of a `/PoseArray` message. The output is `drive_parameters_topic` which is a custom message in the `slip_control_communications` package. The computed steering and velocity commands are sent to the `serial_transmitter.py` node, which maps the output for PWM signals.

More information on the [working of Pure-pursuit controller](https://github.com/KTH-SML/SML_summerproject/blob/master/control1/Purepursuit.md).

The PID controller can be used to control the velocity of the car. For cruise control, the velocity estimates needs to be accurate, which is not always the case while using the GPS. So the usage of cruise control is advised only when the velocity estimates are quite accurate.
