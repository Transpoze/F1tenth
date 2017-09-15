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
In this project (SML Summer job 26.6.2017-6.9.2017), a ROS package named **obstacle_detection** is created for the f1tenth vehichle to avoid single obstacle on its path. Any solid object will be regarded as obstacle without distinction. If several obstacles exist in the detection region, they will be combined and considered as one large obstacle. In general, the package works as a ROS node which reads in point cloud data from ZED camera and sends commands, i.e. a new fixed waypoint, to the controller to bypass the obstacle if there is any. We accomplished such avoidance in two steps: detection and dynamic path planning. The main script is [obstacle_avoidance.py]. Please see details below.
### Obstacle Detection
Our node subscribes to the topic `/zed/point_cloud/cloud_registered` to retrieve standard point cloud data in ros message type [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) from ZED. For the sake of computation cost, the point cloud is evenly sampled with a constant sampling step by calling python script *[readpoint.py]*. Also, a dynamic detection region ahead of the camera was set up to reduce not only computation time but also false detection rate. Any point outside this region will be discarded. The range, i.e. width and depth, of this region is changing dynamically based on the current x and y direction velocity of the vehicle with respect to the local coordinate system. This part is realized in the function *adjust_range()*.

Obstacle(s) is detected when the number of points in the detection region exceeds an empirically pre-determined threshold called *point_count_min*. The threshold increases if you reduce the sampling step. As soon as an obstacle is detected, a flag is set as true. Such detection logic works fine for all kinds of solid objects including human beings, boxes, small hills aside the road, etc. However, according to experiments, detection is not robust when the obstacle is within 0.5 meters and slightly suffers from the illumination. When the obstacle is too close, the dection result jumps between true and false. And, when the obstacle is very bright under the sun, it's more difficult to make correct detection result.
### Dynamic Path Planning
If nothing is detected, the controller works in "*normal mode*", which applys pure pursuit for motion planning. When an obstacle(s) is detected, which essentially means that the number of points in the point cloud is too large to be a false detection, it triggers dynamic path planning. Our node will send a new fixed waypoint aside the obstacle (we use the word "fixed" to indicate that this point is fixed in the utm frame), which is in the utm frame, to the controller. This new waypoint is generated basing on the estimated distance and width of the obstacle.

As shown in the figure below, the vehicle is represented by a black square and the point cloud is represented by a black curve in front of the vehicle. As the car moves in 2-D, so we simplified the problem in a 2-D local coordinate system. The origin indicates the location of the car. We take the closest point to the car from the pre-processed point cloud to measure the distance of the obstacle, and the edge points to indicate the boundary of obstacle, hence we have *L* and *R* which shows the width and distance of the obstacle.

![image](https://github.com/xxxx.jpg)

Then we calculate the left and right open angle as *θ_L* and *θ_R*. The new waypoint *P* will be set on the larger open angle side to guide the car avoiding the obstacle. In this figure, *θ_L* is larger than *θ_R*, so *P* is set on the left side. *OP* is the angle bisector of *θ_L*. The exact location of *P* is given by *θ_P* and *S_P*. *S_P* is a constant variable, which is given by experiments.

There are three things you should bear in mind:
* If *θ_P* is too small, then the vehicle will probably crash on the edge of the obstacle. If the it's too large, then the vehicle will go too far from the original path. To reach a balance, here we use *θ_L* and *θ_R* as the open angle instead of *θ_L*+*θ_0* and *θ_R*+*θ_0*. The base angle *θ_0* is a positive constant variable chosen empirically to limit *θ_P* to a relatively smaller value provided that the vehicle won't run into the obstacle.
* The position of each dot in point cloud is given in the camera frame, which is different from the local frame here in the figure. Hence, in order to calculate the distances and angles mentioned above, we need to do a simple coordinate transformation for the points based on the geometry information of the car, which is accomplished explicitly in the code.
* If the obstacle is about in the middle, there are some randomness for the car to determine which direction to go. Experiments show that the wheels switch quickly between turning left and right, which eventually leads to a crash. To prevent this, an additional condition is that if the difference between *θ_L* and *θ_R* is less than 10 degree, than the car will choose the direction closer to the last look ahead point. For example, suppose the last look ahead point is *D*, then the final *OP* should yield smaller angle between *OP* and *OD*.
* A intuitive concern is that outliers scattering around the real obstacle like noise will make the detection over estimate the size of the obstacle. We apply KMeans clustering on the pre-processed point cloud xyz coordinates with `K=2` and check if the number of points in one cluster is larger than 10 times the other cluster. The smaller cluster will be considered as outlier and thus neglected. However, we tested with single obstacle ahead, usually a box or a human, for ten times, it never detected outliers. The point cloud quality is good and stable, also, the sub-sampling step helps to get rid of the noise.


After the detection and avoidance step, the node will publish the flag, indicating if there is an obstacle or not, and the *θ_P*, indicating the location where the vehicle should go, to the ROS topic `/detect_result` as a customized ROS message. The message type is `Cmd` and `queue_size=1`.

To sum up, parameters you need to care about in the *obstacle_detection* package is that:
'''python
# Initialize obstacle detection parameters
step = 10  # sample the cloud points every step length in both height and width direction
depth_min = 0.1  # to get rid of (0,0,0,0) points
depth_max = 1.5  # valid detection depth is 3 meters from the camera, depth_max = velocity_max * 2* processing_time + slip_distance
height_min = -0.05  # only consider points higher than 0.05 meters
width_min = -0.62
width_max = 0.5
alpha_x = 3  # coefficient for dynamically change the detection range
alpha_y = 3  # coefficient for dynamically change the detection range
interval = 1  # sampling interval for visualizing a subset of clustered cloud points in 3D
k = 2  # number of clusters for KMeans, maximum 8 for visualization convenience
point_count_min = 700/step**2  # minimum number of points to be regarded as a valid obstacle
dis_center_threshold = 1  # min value of distance between two valid cluster, in meters
dis_edge_threshold = 1  # min value of distance between two edges tolerate the car to go through
target_angle = np.pi/2  # default value for current target angle
'''

The node run in at least 10Hz. Time complexity is O(n), n is the amount of points in detection region. There is extra latency caused by camera. It takes around 0.1 seconds between obtaining the image and publishing the point cloud ROS message. In this case, without any deceleration setup, the car should run in a constant speed less than 1.5 m/s. Otherwise, the avoidance is very likely to fail.


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
