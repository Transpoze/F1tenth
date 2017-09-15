#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from obstacle_detection.msg import Cmd, input_model
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
import readpoint as rpt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import cv2
import time

# Define the publisher
result_pub = rospy.Publisher("/detect_result", Cmd, queue_size=1)  # publish detection result
bool_pub = rospy.Publisher("/eStop", Bool, queue_size=1)  # publish command to stop the car

# Initialize obstacle detection parameters
step = 10  # sample the cloud points every step length in both height and width direction
# coefficients for detection region
depth_min = 0.3  
depth_max = 2.5
height_min = 0.1
width_min = -0.62
width_max = 0.5
depth_max_static = 2
width_static = 0.4
base_line = 0.12  # camera base line
alpha_x = 1  # coefficient for dynamically change the detection range
alpha_y = 1  # coefficient for dynamically change the detection range
# coefficients for KMeans clustering
k = 2  # number of clusters for KMeans, maximum 8 for visualization convenience
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.05)
attempts = 10
flag_kmeans = cv2.KMEANS_PP_CENTERS
# detection threshold, minimum number of points to be regarded as a valid obstacle
point_count_min = 700/step**2  
# Initialize obstacle avoidance parameters
theta_zero = 0.36  # in radius, helps to limit the turning angle to an appropriate small value when the vehicle sees an obstacle
# Others
target_angle = np.pi/2  # default value for current target angle
flag = "False"
# MODE = 'DETECTION_ONLY'  # the vehicle only detects and stops if there is an obstacle, no avoidance
MODE = 'AVOIDANCE'  # the vehicle tries to avoid the obstacle if there is any


# Publish bool_msg to /eStop, i.e. the motor,  for stopping
def pub_eStop(boolean_flag):
    bool_msg = Bool()
    bool_msg.data = boolean_flag
    bool_pub.publish(bool_msg)


# Ndarray data normalization
def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2 == 0] = 1
    return a / np.expand_dims(l2, axis)


# Detect if there is obstacle ahead
def detect_3d(cloud):
    # Read the cloud points into list
    points_array = np.asarray([p for p in rpt.read_points(cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True, sample_step=step) if depth_min < p[0] < depth_max and p[2] > height_min and width_min < p[1] < width_max])
    
    return points_array.shape[0], points_array
    # return count, points_array[:count, :]


# Cluster the point cloud
def cluster_3d(data, point_num):
    # Perform clustering on point cloud
    # print "Performing kmeans clustering..."

    # Normalize the last column, i.e. rgb, of the point cloud data ndarray
    point_ndarray_norm = np.concatenate((data[:, :3],
                                         (normalized(data[:, 3], 0)).reshape([point_num, 1])), axis=1)
    
    compactness, labels, centers = cv2.kmeans(np.float32(point_ndarray_norm), k, criteria, attempts, flag_kmeans)

    return centers, labels, compactness  # all np.ndarray


# Perform detection on point cloud
def detect_obstacle(cloud):
    # Initialize default output commands
    detect_result = Cmd()
    global flag
    distance = 0.0
    angle = np.pi/2

    start_time = time.time()
    print "****************************************************"
    print "Current time: ", start_time
    print ("Subscribing to /zed/point_cloud/cloud_registered.\n"
           "Received frame No.%d." % cloud.header.seq)
    print "Time stamp: ", cloud.header.stamp
    # print "Beginning of detection: ", time.time()

    # Detect if there is obstacle
    point_count, point_ndarray = detect_3d(cloud)
    # print "Number of points detected: ", point_count
    # print "For loop over cloud point time: ", time.time() - start_time
    print "Number of points: ", point_count, "****************"
    # Reset the flag
    if point_count < point_count_min:
        flag = "False"
        # detect_result.flag = flag
        # print "No obstacle detected!"

        if MODE == 'DETECTION_ONLY':
            pub_eStop(False)

    else:
        flag = "True"

        print "****************************"
        print "Obstacle detected!"
        print "****************************"

        if MODE == 'DETECTION_ONLY':
            pub_eStop(True)
        elif MODE == 'AVOIDANCE':
            # ---- Dynamic Path Planning ---- #
            # Mehtod 0, no clustering 
            direction = pick_direction(point_ndarray)
            '''
            # Method 1, KMeans clustering with k = 2 for outlier detection
            centers, labels, compactness = cluster_3d(point_ndarray, point_count)
            # Concatenate coordinates with labels
            point_ndarray = np.concatenate((point_ndarray[:, :3], labels), axis=1)
            # Check if the clusters are too close
            dummy, counts = np.unique(point_ndarray[:, 3], return_counts=True)  # counts are sorted based on unique values
            ratio = float(counts[0]) / counts[1]
            # Get rid of the outliers
            if ratio < 0.1 or ratio > 10:
                direction = pick_direction(point_ndarray[point_ndarray[:, 3] == np.argmax(counts)])
            else:
                direction = pick_direction(point_ndarray)
            '''
            angle = direction

    # Publish result
    # flag == "False" --> No obstacle detected
    # flag == "True" & distance == 0 --> Detect obstacle for the first time
    # flag == "True" & distance != 0 --> Already detected obstacle
    # print "Time starting to publish detection result: ", time.time()
    detect_result.flag = flag
    detect_result.distance = distance
    detect_result.turn_angle = np.pi/2 - angle
    # print "===================", np.pi/2 - angle
    result_pub.publish(detect_result)
    # print "Time finishing publishing detection result: ", time.time()


def pick_direction(point_ndarray):
    x = np.min(point_ndarray[:, 0])
    left_y = np.max(point_ndarray[:, 1])
    right_y = np.min(point_ndarray[:, 1])
    left_angle = cal_direction(np.sin(theta_zero) - 0.3, np.cos(theta_zero) - 0.06, x, left_y)
    right_angle = cal_direction(x, right_y, np.sin(theta_zero) - 0.3, -np.cos(theta_zero) - 0.06)
    # print "x: ", x
    # print "ly: ", left_y
    # print "ry: ", right_y
    # print "left angle: ", left_angle
    # print "right angle: ", right_angle
    if abs(left_angle + right_angle - np.pi) > 0.17:
        if left_angle > (np.pi - right_angle):
            return left_angle
        else:
            return right_angle
    else:
        if abs(left_angle - target_angle) < abs(right_angle - target_angle):
            return left_angle
        else:
            return right_angle


# Helper function to transfer coordination system and calculate the direction between two lines
def cal_direction(left_x, left_y, right_x, right_y):
    # offsets between camera frame and vehicle's local frame, basically shift the origin
    left_x += 0.3
    right_x += 0.3
    left_y += 0.06
    right_y += 0.06
    # calculate the open angle
    alpha_left = arctan(left_x/left_y)
    alpha_right = arctan(right_x/right_y)
    return (alpha_left + alpha_right)/2


# Helper function to calculate arctan angle within [0,pi]
def arctan(val):
    rad = np.arctan(val)
    if rad < 0:
        rad += np.pi
    return rad


# Dynamically adjust the detection range w.r.t the current velocity
def adjust_range(odom):
    global depth_max
    global width_min
    global width_max
    v_x = abs(odom.twist.twist.linear.x)  # absolute velocity in x (positive forward), m/s
    v_y = abs(odom.twist.twist.linear.y)  # absolute velocity in y (positive on the right), m/s
    depth_max = depth_max_static + alpha_x * v_x
    width_min = - width_static - alpha_y * v_y
    width_max = - base_line - width_min
    # print "Time stamp of adjusting detection range:", time.time()
    # print "Depth: ", depth_max, "v_x: ", v_x
    # print "Width: ", - width_min, "v_y: ", v_y


def get_target_angle(angle):
    global target_angle
    target_angle = np.pi/2 - angle.data
    print "target_angle: ", angle.data


def image_io_node():
    rospy.init_node('image_io_node', anonymous=True)
    
    if MODE == 'DETECTION_ONLY':
        bool_msg1 = Bool()
        bool_msg1.data = False
        rospy.sleep(1)
        bool_pub.publish(bool_msg1)

    rospy.Subscriber("odometry/filtered", Odometry, adjust_range)
    if MODE == 'AVOIDANCE':
        rospy.Subscriber("target_angle", Float32, get_target_angle)
    rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, detect_obstacle)
    rospy.spin()


if __name__ == '__main__':
    image_io_node()
