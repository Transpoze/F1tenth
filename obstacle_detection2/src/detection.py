#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from obstacle_detection.msg import Cmd
# import sensor_msgs.point_cloud2 as pc2
import readpoint as rpt
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import cv2
from sklearn.cluster import KMeans
import time
# import os
# import threading


# Define the publisher
result_pub = rospy.Publisher("/detect_result", Cmd, queue_size=3)

# Initialize obstacle detection parameters
step = 5  # sample the cloud points every step length in both height and width direction
depth_max = 1.8  # only consider points within 4 meters from the camera, depth_max = velocity_max * processing_time
height_min = 0.05  # only consider points higher than 0.05 meters
width_min = -0.62
width_max = 0.5
interval = 5  # sampling interval for visualizing a subset of clustered cloud points in 3D
k = 4  # number of clusters for KMeans, maximum 8 for visualization convenience
point_count_min = 500/step**2  # minimum number of points to be regarded as a valid obstacle
flag = "True"
# # Initialize the figures
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X - Depth')
# ax.set_ylabel('Y - Width')
# ax.set_zlabel('Z - Height')
# plt.ion()
# plt.show()


# Visualize Clustering result
def visual_3d(point3D):
    # print "Current thread: ", threading.current_thread()
    # print "Main thread: ", threading._MainThread()

    for pt in point3D:
        x = pt[0]
        y = pt[1]
        z = pt[2]
        c = colormap(pt[3])
        ax.scatter(x, y, z, c=c, marker='o')

    plt.draw()
    print "pausing"
    plt.pause(10)


# colormap for plotting 3D clusters
def colormap(c):
    return {
        int(0): 'r',
        int(1): 'g',
        int(2): 'b',
        int(3): 'y',
        int(4): 'c',
        int(5): 'm',
        int(6): 'k',
        int(7): 'w',
    }[c]


# Ndarray data normalization
def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2 == 0] = 1
    return a / np.expand_dims(l2, axis)


# Detect if there is obstacle ahead
def detect_3d(cloud, stop_sign):
    # Read the cloud points into list
    points_array = np.zeros([cloud.height * cloud.width, 4])
    count = 0
    for p in rpt.read_points(cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True, sample_step=step):
        # "x: %f\ny: %f\nz: %f\nrgb: %f" % (p[0], p[1], p[2], p[3])
        # coordinates system relative to /zed_current_frame, basing on the left camera
        # x: depth, y: horizontal distance to the opposite of right camera, z: height
        if p[0] < depth_max and p[2] > height_min and width_min < p[1] < width_max:
            if stop_sign:
                points_array[count, :] = np.asarray(p)
            count += 1

    print "For loop over cloud point time: ", time.time()
    return count, points_array[:count, :]


# Cluster the point cloud
def cluster_3d(data, point_num):
    # Perform clustering on point cloud
    # print "Performing kmeans clustering..."

    # Normalize the last column, i.e. rgb, of the point cloud data ndarray
    point_ndarray_norm = np.concatenate((data[:, :3],
                                         (normalized(data[:, 3], 0)).reshape([point_num, 1])), axis=1)

    # # Clustering using sklearn.KMeans
    # obstacles = KMeans(n_clusters=k, n_jobs=1).fit(point_ndarray_norm)
    # labels = obstacles.labels_.reshape([point_num, 1])
    # centers = obstacles.cluster_centers_
    # compactness = None

    # Clustering using cv2.kmeans
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.05)
    attempts = 10
    flag_kmeans = cv2.KMEANS_PP_CENTERS
    compactness, labels, centers = cv2.kmeans(np.float32(point_ndarray_norm), k, None, criteria, attempts, flag_kmeans)

    return centers, labels, compactness  # both np.ndarray


# Perform detection on point cloud
def detect_obstacle(cloud):
    # Initialize default output commands
    detect_result = Cmd()
    global flag
    distance = 0.0
    angle = 0.0

    print "****************************************************"
    print ("Subscribing to /zed/point_cloud/cloud_registered.\n"
           "Received frame No.%d." % cloud.header.seq)
    print "Current time: ", time.time()

    # Detect if there is obstacle
    point_count, point_ndarray = detect_3d(cloud, flag)
    print "Number of points detected: ", point_count

    # Reset the flag
    if point_count < point_count_min:
        flag = "False"
        detect_result.flag = flag
        print "No obstacle detected!"
    else:
        if flag == "False":
            flag = "True"
            print "Obstacle detected!"
        else:
            print "Obstacle detected!"
            a = time.time()
            # Clustering
            centers, labels, compactness = cluster_3d(point_ndarray, point_count)
            b = time.time()
            print "Clustering time: ", b - a

            # Print out the clustering result
            print "Cluster compactness: ", compactness
            print "Centroids: [x:depth, y:width, z:height, rgb:rgb]\n", centers

            ####################### Path planning ########################

            # # Concatenate coordinates with labels to visualize clustering result
            # point_clustered_ndarray = np.concatenate((point_ndarray[:, :3], labels),
            #                                          axis=1)
            # np.random.shuffle(point_clustered_ndarray)
            #
            # # # Plot the clusters in 3D
            # # visual_3d(point_clustered_ndarray[0::interval, ...])
            # # # plt.show(block=True)
            #
            # # Save the clusters locally
            # # cwd = os.getcwd()
            # cwd = "/home/xiao/ros/catkin_ws/src/obstacle_detection/src"
            # np.savetxt(cwd + '/clustered_cloud_points.txt', point_clustered_ndarray[0::interval, ...])  # saved under ~/
            # print "Successively saved point_clustered_ndarray"

    # Publish result
    # flag == "False" --> No obstacle detected
    # flag == "True" & distance == 0 --> Detect obstacle for the first time
    # flag == "True" & distance != 0 --> Already detected obstacle
    detect_result.flag = flag
    detect_result.distance = distance
    detect_result.turn_angle = angle
    result_pub.publish(detect_result)


def image_io_node():
    rospy.init_node('image_io_node', anonymous=True)
    # rate = rospy.Rate(25) # 25hz, depends on how fast images are sent by zed camera
    # zed_depth_topic = "/zed/depth/depth_registered"
    zed_cloud_point_topic = "/zed/point_cloud/cloud_registered"
    # depth_msg_type = Image
    cloud_point_msg_type = PointCloud2
    rospy.Subscriber(zed_cloud_point_topic, cloud_point_msg_type, detect_obstacle)
    rospy.spin()


if __name__ == '__main__':
    image_io_node()
