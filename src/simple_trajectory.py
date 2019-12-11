#!/usr/bin/python2
import rospy
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt

import numpy as np
import math
from scipy.interpolate import splprep, splev,interp1d

_map_loaded = False
_map = []
_trajectory_points = []
_trajectory_done = False

"""
This script creates the vehicle trajectory based on points marked in the map in RVIZ 
"""

pnt_pub = rospy.Publisher('trajectory_points', Marker, queue_size=10)
path_pub = rospy.Publisher('reference_path/path', Path, queue_size=10)
pathm_pub = rospy.Publisher('reference_path/marker', Marker, queue_size=10)

def simple_trajectory():

    global _trajectory_points

    x, y = _trajectory_points.T
    i = np.arange(len(_trajectory_points))

    # 5x the original number of points
    interp_i = np.linspace(0, i.max(), 80 * i.max())

    xi = interp1d(i, x, kind='cubic')(interp_i)
    yi = interp1d(i, y, kind='cubic')(interp_i)

    poses = []
    ln = xi.shape
    ln = ln[0]
    print(ln)

    pthm = Marker()
    pthm.header.frame_id = 'map'
    pthm.type = 8
    pthm.scale.x = 0.05
    pthm.scale.y = 0.05
    pthm.color.a = 1.0
    pthm.color.r = 0.1
    pthm.color.g = 1.0
    pthm.color.b = 0.2
    pthm.points = []

    for i in range(0,ln):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = xi[i]
        ps.pose.position.y = yi[i]
        ps.pose.position.z = 0
        poses.append(ps)

        p = Point()
        p.x = xi[i]
        p.y = yi[i]
        p.z = 0
        pthm.points.append(p)

    pth = Path()
    pth.header.frame_id = 'map'
    pth.poses = poses

    del pthm.points[-1]

    # Publisher
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
        path_pub.publish(pth)
        pathm_pub.publish(pthm)

    return

def map_callback(map):

    global _map_loaded

    if not _map_loaded:
        occupancy_grid = np.array(map.data).reshape((map.info.height, map.info.width))
        resolution = map.info.resolution
        width = map.info.width
        height = map.info.height
        occupancy_grid_blown = np.zeros((width, height))

        # Occupancy grid legend:
        # -1 = unexplored
        # 0 = free
        # 100 = obstacle
        for row in range(0,height):
            for col in range(0,width):
                pass

        # Sort
        for row in range(0,height):
            for col in range(0,width):
                break

        _map_loaded = True

    return

def clicked_point(data):
    """
    Callback function for clicked point in rviz
    :return:
    """
    global _trajectory_done, _trajectory_points

    if _trajectory_done:
        return

    if len(_trajectory_points) > 2:
        first_point = _trajectory_points[0]
        dist = math.sqrt(math.pow(first_point[0]-data.point.x,2) + math.pow(first_point[1]-data.point.y,2))
        print(dist)
        if dist < 0.15:
            print("trajectory done")
            _trajectory_done = True
            _trajectory_points = np.vstack((_trajectory_points, _trajectory_points[0]))
            simple_trajectory()
            return

    if len(_trajectory_points) == 0:
        # First element
        _trajectory_points = np.array([[data.point.x,data.point.y]])
    else:
        _trajectory_points = np.vstack((_trajectory_points, np.array([data.point.x,data.point.y])))

    print(_trajectory_points)

    #Marker message
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15

    points = []
    colors = []

    for p in _trajectory_points:
        pnt = Point()
        pnt.x = p[0]
        pnt.y = p[1]
        pnt.z = 0
        points.append(pnt)
        colors.append(ColorRGBA(1,0,0,1))

    marker.colors = colors
    marker.points = points
    pnt_pub.publish(marker)

if __name__ == "__main__":

    rospy.init_node('simple_trajectory', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber("clicked_point", PointStamped, clicked_point)
    rospy.spin()
