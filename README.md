# SML_summerproject

A ROS-package developed during summer 2017 for localization and control of the F1Tenth car. 



## Localizaton 
### Sensors
All sensors are handeled by the `sensor_helpers` package. To launch the sensors (together with`serial_node` for the Teensy board) use `roslaunch sensor_helpers launch_sensors.launch`. Note that you might need to adjust the port names in the launch file. 

There are currently two sensors being used for localization:
- Razor IMU 9DoF - accelerometer, gyro and magnetometer data avaliable as an [Imu message](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) on `/imu`
- Emlid Reach RTK kit - see [reach instructions](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_localization/reach_instructions.md) for more info

### Sensor fusion
Sensor fusion is handeled by the `f1tenth_localization` package. Incomming information from the sensors is fused using an Extended Kalman Filter (EKF) implemented in the ROS-package [robot_localization](http://docs.ros.org/kinetic/api/robot_localization/html/index.html). You can run the filter using `roslaunch f1tenth_localization ekf_gps_imu.launch`.

The EKF calculations take place in an instance of `ekf_localization_node` that estimates the state of the car and publishes the information as an [odometry message](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) to `/odometry/filtered`. There is also an instance of `navsat_transform_node` that transforms latitude/longitude measurements from the Reach into local coordinates. 

#### Frames
- the position is given in the `odom` frame which has its origin at the starting location of the car with the positive x-axis pointing north
- the velocity is given in the `base_link` frame which is rigidly mounted on the base of the car (i.e. `odom.twist.twist.linear.x` is always forward velocity etc)


The `odom->base_link` map is provided by the `ekf_localization_node` 

Internally, `navsat_transform_node` uses the [utm coordinate system](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) and a `utm->odom` transform is provided by the node. For converting between utm and latitude/lingitude you can use the python-package [utm](https://pypi.python.org/pypi/utm).
