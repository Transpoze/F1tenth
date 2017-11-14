import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

'''
This script should be in the same directory as detection.py
to successfully load saved cloud point txt file
'''


# Visualize Clustering result
def visual_3d(point3D):
    fig = plt.figure()
    clustered_cloud = fig.add_subplot(111, projection='3d')

    for pt in point3D:
        x = pt[0]
        y = pt[1]
        z = pt[2]
        c = colormap(pt[3])
        clustered_cloud.scatter(x, y, z, c=c, marker='o')

    clustered_cloud.set_xlabel('X - Depth')
    clustered_cloud.set_ylabel('Y - Width')
    clustered_cloud.set_zlabel('Z - Height')

    plt.show()


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


if __name__ == '__main__':
    cwd = os.getcwd()
    pts = np.loadtxt(cwd + '/clustered_cloud_points.txt')
    print type(pts[0, 0])
    print type(pts[0, 1])
    print type(pts[0, 2])
    print type(pts[0, 3])  # all float64, but actually perform clustering on float32
    visual_3d(pts)
