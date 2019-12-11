#!/usr/bin/python2
# simple_trajectory.py
"""This ROS node generates a trajectory from points selected using rViz.

                          ------------
                         | map server |
                          ------------
                               |
                           map | OccupancyGrid
                               v
                          ------------
 ------   clicked_point  |   simple   |  reference_path/{path, marker}
| rViz | --------------> |            | ------------------------------>
 ------   PointStamped   | trajectory |        {Path, Marker}
                          ------------
                               |
             trajectory_points | Marker
                               v
                            ------
                           | rViz |
                            ------
"""
######################
# Imports & Globals
######################

# ROS python package
import rospy

# Math engine
import numpy as np
import math

# Math engine / interpolation
from scipy.interpolate import splprep, splev, interp1d

# Plot
import matplotlib.pyplot as plt


# Message types
# ColorRGBA
from std_msgs.msg import ColorRGBA
#: float32 r
#: float32 g
#: float32 b
#: float32 a

# Header
from std_msgs.msg import Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
#
# sequence ID: consecutively increasing ID
#: uint32 seq
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
#: time stamp
# Frame this data is associated with
# 0: no frame
# 1: global frame
#: string frame_id

# MapMetaData
from nav_msgs.msg import MapMetaData
# This hold basic information about the characterists of the OccupancyGrid
#
# The time at which the map was loaded
#: time map_load_time
# The map resolution [m/cell]
#: float32 resolution
# Map width [cells]
#: uint32 width
# Map height [cells]
#: uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
#: geometry_msgs/Pose origin

# Marker
from visualization_msgs.msg import Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and
# http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
# for more information on using this message with rviz
#
#: uint8 ARROW=0
#: uint8 CUBE=1
#: uint8 SPHERE=2
#: uint8 CYLINDER=3
#: uint8 LINE_STRIP=4
#: uint8 LINE_LIST=5
#: uint8 CUBE_LIST=6
#: uint8 SPHERE_LIST=7
#: uint8 POINTS=8
#: uint8 TEXT_VIEW_FACING=9
#: uint8 MESH_RESOURCE=10
#: uint8 TRIANGLE_LIST=11
#
#: uint8 ADD=0
#: uint8 MODIFY=0
#: uint8 DELETE=2
#: uint8 DELETEALL=3
#
#: Header header                        # header for time/frame information
#: string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
#: int32 id                             # object ID useful in conjunction with the namespace for manipulating and deleting the object later
#: int32 type                           # Type of object
#: int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
#: geometry_msgs/Pose pose              # Pose of the object
#: geometry_msgs/Vector3 scale          # Scale of the object 1,1,1 means default (usually 1 meter square)
#: std_msgs/ColorRGBA color             # Color [0.0-1.0]
#: duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
#: bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
#
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#: geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
# number of colors must either be 0 or equal to the number of points
# NOTE: alpha is not yet used
#: std_msgs/ColorRGBA[] colors
#
# NOTE: only used for text markers
#: string text
#
# NOTE: only used for MESH_RESOURCE markers
#: string mesh_resource
#: bool mesh_use_embedded_materials

# OccupancyGrid
from nav_msgs.msg import OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.
#
#: Header header
#
# MetaData for the map
#: MapMetaData info
#
# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
#: int8[] data

# Path
from nav_msgs.msg import Path
#An array of poses that represents a Path for a robot to follow
#: Header header
#: geometry_msgs/PoseStamped[] poses

# Point
from geometry_msgs.msg import Point
# This contains the position of a point in free space
#: float64 x
#: float64 y
#: float64 z

# PointStamped
from geometry_msgs.msg import PointStamped
# This represents a Point with reference coordinate frame and timestamp
#: Header header
#: Point point

# Pose
from geometry_msgs.msg import Pose
# A representation of pose in free space, composed of position and orientation.
#: Point position
#: Quaternion orientation

# PoseStamped
from geometry_msgs.msg import PoseStamped
# A Pose with reference coordinate frame and timestamp
#: Header header
#: Pose pose


# Global variables
_map_loaded = False
_map = []
_trajectory_points = []
_trajectory_done = False


# Publishers
pnt_pub = rospy.Publisher('trajectory_points', Marker, queue_size=10)
path_pub = rospy.Publisher('reference_path/path', Path, queue_size=10)
pathm_pub = rospy.Publisher('reference_path/marker', Marker, queue_size=10)


######################
# Utilities
######################

def simple_trajectory():
    """Interpolate received points by a cubic curve.

    Note: Calling this will block the callback.
    """
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


######################
# Callbacks
######################

def map_callback(map):
    """Save map of the environment for later usage.

    Arguments:
    map -- map of the environment, defined by nav_msgs.msg/OccupancyGrid

    Note: This is not used at all. Maybe in the future it should check that
    the generated trajectory is within the bounds.
    """
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
    """Register a point passed using rViz 'Publish Point' function.

    Arguments:
    data -- received point, defined by geometry_msgs.msg/PointStamped
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


######################
# Functions
######################

def start_node():
    """Starts a ROS node, registers the callbacks."""

    # Let only one node run
    rospy.init_node('simple_trajectory', anonymous=False)

    # Register callback
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber("clicked_point", PointStamped, clicked_point)

    # Function is_shutdown() reacts to exit flag (Ctrl+C, etc.)
    while not rospy.is_shutdown():

        # Function spin() simply keeps python from exiting until this node is stopped.
        rospy.spin()


if __name__ == "__main__":
    start_node()
