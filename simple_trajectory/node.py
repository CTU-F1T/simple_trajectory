#!/usr/bin/python2
# node.py
"""This ROS node generates a trajectory from points selected using rViz.

                          ------------
                         | map server |
                          ------------
                               |
                           map | OccupancyGrid
                               v
                          ------------
 ------   clicked_point  |            |  reference_path/{path, marker}
| rViz | --------------> |            | ------------------------------>
 ------   PointStamped   |   simple   |        {Path, Marker}
                         |            |
 ---------     path      | trajectory |    reference_path/gridcells
| storage | ...........> |            | ------------------------------>
 ---------     Path      |            |           GridCells
                          ------------
                               |
             trajectory_points | Marker
                               v
                            ------
                           | rViz |
                            ------

This node is used to create a custom path using 'Publish Point' feature
that is available in rViz. When a global map is provided, it is used
to compute surrounding area of the path to define all valid positions.

Using 'Publish Point' you can place path via-points. They are published
on topic 'trajectory_points'.

First placed point has purple color, other points are red. When at least
three points are placed, placing a new point on the first (purple) one
sends a signal to close the path, interpolate it using CubicSpline and
publish it. As a note, placed points are interpolated in placing order.

Placing a point near another red point (or purple point in case that
there are only less then three points) colors this point yellow. Now,
there are three options according to where you place the next point:
 - placing it on the yellow point makes it red again as nothing has
   happened,
 - placing it somewhere else moves the yellow point to this location;
   ordering of the points is not changed,
 - placing it on another point deletes the yellow point.

Additionally, placing a point somewhere in the environment leads to
finding two closest via-points on the path, and this new point is
placed in-between of them.
"""
######################
# Imports & Globals
######################

import os

# ROS wrapper
from autopsy.node import Node
from autopsy.core import Core, ROS_VERSION

# Math engine
import numpy

# Math engine / interpolation
from scipy.interpolate import CubicSpline

# Progress printing
import sys
import tqdm

# Dynamic Reconfigure / ParameterServer
from autopsy.reconfigure import ParameterServer

# Loading stored data
import csv

# Transformation of the trajectory
from tf.transformations import (
    quaternion_matrix
)


# Message types
from std_msgs.msg import (
    ColorRGBA,
)
from nav_msgs.msg import (
    GridCells,
    OccupancyGrid,
    Path,
)
from visualization_msgs.msg import (
    Marker,
)
from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    Quaternion,
    Vector3,
)


# Global variables
MAP_LOADED = False
INFLATED_MAP = False
_MAP = None
MAP_HEADER = None
MAP_INFO = None
BOUNDS = None
TRAJECTORY_POINTS = []
TRAJECTORY_DONE = False
TRAJECTORY_DISTANCE = 0
INFLATE_TRAJECTORY = None  # create_surroundings(TRAJECTORY_DISTANCE)
INFLATE_DISTANCE = 0
INFLATE_AREA = None  # create_surroundings(INFLATE_DISTANCE)
PICKING_UP = False
PICK_UP_I = 0
RELOAD_MAP = False
CLOSED_PATH = False
NODE_HANDLE = None  # Node handle, a way how to not rewrite everything


# Parameters
P = ParameterServer()
P.update([
    # Path/Map parameters
    ("trajectory_inflate", {
        "default": 0,
        "min": 0,
        "max": 100,
        "description": "[cells], path inflation radius.",
        "callback": lambda value: reconf_trajectory_inflate(value)
    }),
    ("map_inflate", {
        "default": 0,
        "min": 0,
        "max": 100,
        "description": "[cells], map inflation radius.",
        "callback": lambda value: reconf_map_inflate(value)
    }),
    ("publish_cropped_map", {
        "default": False,
        "description": "Publish internal cropped map.",
        "callback": lambda value: reconf_publish_cropped_map(value)
    }),
    ("reload_map", {
        "default": False,
        "description": "Reload map when new one is received.",
        "callback": lambda value: reconf_reload_map(value)
    }),
    ("path_length", {
        "default": 440,
        "min": 1,
        "max": 2000,
        "description": "[points], number of points in the path.",
    }),
    ("double_interpolation", {
        "default": True,
        "description": "Perform a double interpolation to obtain truly "
                       "equidistant path points.",
    }),

    # Path type
    ("closed_path", {
        "default": False,
        "description": (
            "Type of path received on topic / loaded from a file."
        )
    }),

    # Loading data from a file
    ("input_file", {
        "default": "",
        "description": "Name of a file to load data from.",
        "callback": lambda value: reconf_input_file(value)
    }),
    ("delimiter", {
        "default": "",
        "description": (
            "Delimiter used in the file. When empty, load file as npz. "
            "Note: Use '\,' to set the delimiter to a comma."  # noqa: W605
        )
    }),
])


######################
# Dynamic reconfigure
######################

def reconf_map_inflate(value):
    """Reconfigure callback for 'map_inflate'."""
    global MAP_LOADED, INFLATED_MAP
    global INFLATE_DISTANCE, INFLATE_AREA
    global TRAJECTORY_DISTANCE

    NODE_HANDLE.loginfo("Reconfigure request: map_inflate = %d" % value)

    if MAP_LOADED and value != INFLATE_DISTANCE:

        INFLATE_DISTANCE = value

        INFLATE_AREA = create_surroundings(INFLATE_DISTANCE)

        inflate_map()

        INFLATED_MAP = True

        reconf_trajectory_inflate(TRAJECTORY_DISTANCE)

    return value


def reconf_trajectory_inflate(value):
    """Reconfigure callback for 'trajectory_inflate'."""
    global TRAJECTORY_DONE, INFLATED_MAP
    global TRAJECTORY_DISTANCE, INFLATE_TRAJECTORY

    NODE_HANDLE.loginfo(
        "Reconfigure request: trajectory_inflate = %d" % value
    )

    if TRAJECTORY_DONE and (value != TRAJECTORY_DISTANCE or INFLATED_MAP):
        TRAJECTORY_DISTANCE = value

        INFLATE_TRAJECTORY = create_surroundings(TRAJECTORY_DISTANCE)

        simple_trajectory()

        INFLATED_MAP = False

    return value


def reconf_publish_cropped_map(value):
    """Reconfigure callback for 'publish_cropped_map'."""
    global NODE_HANDLE

    NODE_HANDLE.loginfo(
        "Reconfigure request: publish_cropped_map = %s" % value
    )

    if value and NODE_HANDLE.map_pub is None:
        NODE_HANDLE.map_pub = NODE_HANDLE.Publisher(
            'reference_path/map',
            OccupancyGrid,
            queue_size = 1,
            latch = True
        )
        publish_map()
    elif not value and NODE_HANDLE.map_pub is not None:
        del NODE_HANDLE.map_pub
        NODE_HANDLE.map_pub = None

    return value


def reconf_reload_map(value):
    """Reconfigure callback for 'reload_map'."""
    global RELOAD_MAP

    NODE_HANDLE.loginfo("Reconfigure request: reload_map = %s" % value)

    RELOAD_MAP = value

    return value


def reconf_input_file(value):
    """Reconfigure callback for 'input_file'."""
    NODE_HANDLE.loginfo("Reconfigure request: input_file = %s" % value)

    ret = load_data(
        str(value),
        str(P.delimiter.value)
    )

    if ret:
        return value
    else:
        return P.input_file.value


######################
# Map related functions
######################

def create_surroundings(radius):
    """Create array of surrounding cells.

    Arguments:
    radius -- radius of the surrounding area, number of cells, int
    """
    _area = numpy.array(
        numpy.meshgrid(
            range(-radius, radius + 1),
            range(-radius, radius + 1)
        )
    ).T.reshape(-1, 2)

    return _area[
        numpy.hypot(_area[:, 0], _area[:, 1]) <= radius, :
    ]


def publish_map():
    """Publish inflated cropped map."""
    global MAP_LOADED, MAP_INFLATED, MAP_INFO, MAP_HEADER

    if MAP_LOADED and NODE_HANDLE.map_pub is not None:
        map = OccupancyGrid()
        map.header = MAP_HEADER
        map.header.stamp = NODE_HANDLE.get_clock().now().to_msg()
        map.info = MAP_INFO
        map.data = list(MAP_INFLATED.flatten())

        NODE_HANDLE.map_pub.publish(map)


def inflate_map():
    """Inflate the map and store it."""
    global MAP_LOADED, _MAP, MAP_HEADER, MAP_INFO, MAP_INFLATED

    NODE_HANDLE.loginfo("Inflating map...")

    MAP_INFLATED = numpy.zeros((MAP_INFO.height, MAP_INFO.width))

    map_walls = numpy.where(_MAP == 100)

    # Add walls to the to-be-inflated map
    for _y, _x in zip(map_walls[0], map_walls[1]):
        MAP_INFLATED[_y, _x] = 100


    # Inflate the map
    for _y, _x in tqdm.tqdm(zip(map_walls[0], map_walls[1]), leave = False):
        for __x, __y in INFLATE_AREA:
            if (
                _y + __y >= 0 and _y + __y < MAP_INFO.height
                and _x + __x >= 0 and _x + __x < MAP_INFO.width
            ):
                MAP_INFLATED[_y + __y, _x + __x] = 100

    NODE_HANDLE.loginfo("Map inflated.")

    publish_map()


######################
# Publish utilities
######################

def publish_trajectory_points():
    """Publish current trajectory points."""
    # Marker message
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.pose.orientation = Quaternion(
        x = 0.0, y = 0.0, z = 0.0, w = 1.0
    )
    marker.scale = Vector3(
        x = 0.15, y = 0.15, z = 0.15
    )

    for i, p in enumerate(TRAJECTORY_POINTS):
        marker.points.append(
            Point(
                x = float(p[0]), y = float(p[1]), z = 0.0
            )
        )

        if PICKING_UP and i == PICK_UP_I:
            marker.colors.append(
                # Yellow (currently selected point)
                ColorRGBA(r = 1.0, g = 1.0, b = 0.0, a = 1.0)
            )
        elif not TRAJECTORY_DONE and i == 0:
            marker.colors.append(
                # Magenta (first point of non-finished trajectory)
                ColorRGBA(r = 1.0, g = 0.0, b = 1.0, a = 1.0)
            )
        else:
            marker.colors.append(
                # Red (any other point)
                ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0)
            )

    NODE_HANDLE.pnt_pub.publish(marker)


######################
# Utilities
######################

def simple_trajectory():
    """Interpolate received points by a cubic curve.

    Wrapper that handles closed/unclosed paths.
    """
    global CLOSED_PATH, TRAJECTORY_POINTS

    if CLOSED_PATH:
        TRAJECTORY_POINTS = numpy.vstack(
            (TRAJECTORY_POINTS, TRAJECTORY_POINTS[0])
        )
        _simple_trajectory()
        TRAJECTORY_POINTS = TRAJECTORY_POINTS[0:-1]
    else:
        _simple_trajectory()


def _simple_trajectory():
    """Interpolate received points by a cubic curve.

    Note: Calling this will block the callback.

    Source:
    center_trajectory.py:interpolate_points() by David Kopecky
    https://stackoverflow.com/questions/52014197/how-to-interpolate-a-2d-curve-in-python
    profile_trajectory/profile_trajectory.py:interpolate_points()
    """
    global TRAJECTORY_POINTS, MAP_LOADED, _MAP
    global MAP_INFO, BOUNDS, MAP_INFLATED, CLOSED_PATH

    x, y = TRAJECTORY_POINTS.T
    i = numpy.arange(len(TRAJECTORY_POINTS))

    distance = numpy.cumsum(
        numpy.sqrt(
            numpy.sum(
                numpy.diff(TRAJECTORY_POINTS, axis = 0) ** 2, axis = 1
            )
        )
    )
    distance = numpy.insert(distance, 0, 0) / distance[-1]

    alpha = numpy.linspace(0, 1, P.path_length)

    NODE_HANDLE.loginfo(
        "Interpolating points to obtain %d points..."
        % P.path_length
    )

    spline = CubicSpline(
        distance, TRAJECTORY_POINTS, axis = 0,
        bc_type = ("periodic" if CLOSED_PATH else "not-a-knot")
    )

    # First interpolation, this one is really rough
    ipol = spline(alpha)
    xi = ipol[:, 0]
    yi = ipol[:, 1]

    # Create an alternative spline to obtain arc-length parameterization
    # At first, compute the delta-distance between consecutive points.
    # Note that first and last points are repeated
    ipol_dists = numpy.hypot(xi[1:] - xi[:-1], yi[1:] - yi[:-1])

    NODE_HANDLE.loginfo(
        "First interpolation: min: %f, avg: %f, max: %f"
        % (
            numpy.min(ipol_dists),
            numpy.mean(ipol_dists),
            numpy.max(ipol_dists)
        )
    )


    if P.double_interpolation:
        # Do a cumulative sum to obtain arc length
        ipol_progress = numpy.hstack((0, numpy.cumsum(ipol_dists)))

        # Create a new spline that serves as an arc-length approximation
        # ipol_progress is normalized
        spline_al = CubicSpline(
            ipol_progress / ipol_progress[-1], alpha, axis=0
        )

        # Perform a second interpolation with adapted alpha
        # alpha is now treated as normalized length over the spline
        # spline_al converts alpha to 's'; a spline parameter
        ipol = spline(spline_al(alpha))
        xi = ipol[:, 0]
        yi = ipol[:, 1]

        # First and last points are repeated
        dists = numpy.hypot(xi[1:] - xi[:-1], yi[1:] - yi[:-1])

        NODE_HANDLE.loginfo(
            "Second interpolation: min: %f, avg: %f, max: %f"
            % (numpy.min(dists), numpy.mean(dists), numpy.max(dists))
        )

    ln = xi.shape
    ln = ln[0]

    pth = Path()
    pth.header.frame_id = 'map'

    pthm = Marker()
    pthm.header.frame_id = 'map'
    pthm.type = Marker.POINTS
    pthm.scale = Vector3(
        x = 0.05, y = 0.05, z = 0.0
    )
    pthm.color = ColorRGBA(
        r = 0.1, g = 1.0, b = 0.2, a = 1.0
    )

    for i in range(0, ln):
        _p = Point(
            x = float(xi[i]), y = float(yi[i]), z = 0.0
        )

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position = _p
        pth.poses.append(ps)

        pthm.points.append(_p)


    del pthm.points[-1]

    NODE_HANDLE.path_pub.publish(pth)
    NODE_HANDLE.pathm_pub.publish(pthm)


    # Available surroundings
    # Store path to map and inflate it
    if MAP_LOADED:
        NODE_HANDLE.loginfo("Inflating path...")

        n_map = numpy.zeros((MAP_INFO.height, MAP_INFO.width))

        for i in tqdm.tqdm(range(0, ln), leave = False):
            _x = int(
                (xi[i] - MAP_INFO.origin.position.x) / MAP_INFO.resolution
            )
            _y = int(
                (yi[i] - MAP_INFO.origin.position.y) / MAP_INFO.resolution
            )

            # Inflate
            for __x, __y in INFLATE_TRAJECTORY:
                if (
                    _y + __y >= 0 and _y + __y < MAP_INFO.height
                    and _x + __x >= 0 and _x + __x < MAP_INFO.width
                ):
                    if MAP_INFLATED[_y + __y, _x + __x] == 0:
                        n_map[_y + __y, _x + __x] = 1

        # Color the map and forget all disjoint regions
        # (disjoint from the path)
        for i in range(0, ln):
            _x = int(
                (xi[i] - MAP_INFO.origin.position.x) / MAP_INFO.resolution
            )
            _y = int(
                (yi[i] - MAP_INFO.origin.position.y) / MAP_INFO.resolution
            )

            if 0 <= _y < MAP_INFO.height and 0 <= _x < MAP_INFO.width:
                if n_map[_y, _x] == 1 and MAP_INFLATED[_y, _x] == 0:
                    n_map[_y, _x] = 2

        colored_cells = True

        while colored_cells:
            colored_cells = False

            map_walls = numpy.where(n_map == 1)

            for _j, _i in zip(map_walls[0], map_walls[1]):
                if (
                    (_j > 0 and n_map[_j - 1, _i] == 2)
                    or (_j < MAP_INFO.height - 1 and n_map[_j + 1, _i] == 2)
                    or (_i > 0 and n_map[_j, _i - 1] == 2)
                    or (_i < MAP_INFO.width - 1 and n_map[_j, _i + 1] == 2)
                ):
                    n_map[_j, _i] = 2
                    colored_cells = True

        gc = GridCells()
        gc.header.frame_id = 'map'
        gc.cell_width = MAP_INFO.resolution
        gc.cell_height = MAP_INFO.resolution
        gc.cells = []

        map_walls = numpy.where(n_map == 2)

        # Rotation
        # http://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
        q = numpy.asarray([
            MAP_INFO.origin.orientation.w,
            MAP_INFO.origin.orientation.x,
            MAP_INFO.origin.orientation.y,
            MAP_INFO.origin.orientation.z
        ])
        qi = numpy.asarray([
            MAP_INFO.origin.orientation.w,
            -MAP_INFO.origin.orientation.x,
            -MAP_INFO.origin.orientation.y,
            -MAP_INFO.origin.orientation.z
        ])

        for _j, _i in zip(map_walls[0], map_walls[1]):
            p = numpy.asarray([
                0,
                MAP_INFO.origin.position.x + _i * MAP_INFO.resolution,
                MAP_INFO.origin.position.y + _j * MAP_INFO.resolution,
                0
            ])

            pr = multiply_quaternions(multiply_quaternions(qi, p), q)

            gc.cells.append(
                Point(
                    x = float(pr[1]), y = float(pr[2]), z = float(pr[3])
                )
            )

        NODE_HANDLE.infgc_pub.publish(gc)

        NODE_HANDLE.loginfo("Path inflated.")

    return


def multiply_quaternions(r, s):
    """Multiply two wxyz-quaternions.

    Arguments:
    r -- first quaternion for multiplication, 4-numpy.ndarray
    s -- second quaternion for multiplication, 4-numpy.ndarray

    Returns:
    t -- result of the product of quaternions, 4-numpy.ndarray
    """
    return numpy.asarray([
        r[0] * s[0] - r[1] * s[1] - r[2] * s[2] - r[3] * s[3],
        r[0] * s[1] + r[1] * s[0] - r[2] * s[3] + r[3] * s[2],
        r[0] * s[2] + r[1] * s[3] + r[2] * s[0] - r[3] * s[1],
        r[0] * s[3] - r[1] * s[2] + r[2] * s[1] + r[3] * s[0]
    ])


def list_intersection(list_a, list_b):
    """Intersect two lists and find common elements.

    Source:
    https://www.geeksforgeeks.org/python-intersection-two-lists/
    """
    return [value for value in list_a if value in list_b]


def list_contains(list_a, list_b):
    """Check whether all elements if list_a are in the list_b."""
    return len(list_a) == len(list_intersection(list_a, list_b))


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
    global MAP_LOADED, _MAP, MAP_HEADER, MAP_INFO, BOUNDS, MAP_INFLATED
    global RELOAD_MAP

    if MAP_LOADED and not RELOAD_MAP:
        return

    # Load map
    _MAP = numpy.array(map.data).reshape((map.info.height, map.info.width))

    # Save info
    MAP_INFO = map.info
    MAP_HEADER = map.header

    # Cut the map
    # Find first and last occurence of a non-(-1) value
    x_lim = numpy.take(
        numpy.where(numpy.max(_MAP, axis=0) != -1)[0],
        [0, -1]
    )

    y_lim = numpy.take(
        numpy.where(numpy.max(_MAP, axis=1) != -1)[0],
        [0, -1]
    )

    _MAP = _MAP[y_lim[0]:y_lim[1], x_lim[0]:x_lim[1]]


    # Update metadata
    MAP_INFO.height = _MAP.shape[0]
    MAP_INFO.width = _MAP.shape[1]
    MAP_INFO.origin.position.x += x_lim[0] * map.info.resolution
    MAP_INFO.origin.position.y += y_lim[0] * map.info.resolution

    # xmin, xmax; ymin, ymax
    BOUNDS = [
        MAP_INFO.origin.position.x,
        MAP_INFO.origin.position.x + MAP_INFO.width * map.info.resolution,
        MAP_INFO.origin.position.y,
        MAP_INFO.origin.position.y + MAP_INFO.height * map.info.resolution
    ]

    NODE_HANDLE.logdebug(BOUNDS)

    # Inflate map
    inflate_map()

    MAP_LOADED = True
    publish_map()


def clicked_point(data):
    """Register a point passed using rViz 'Publish Point' function.

    Arguments:
    data -- received point, defined by geometry_msgs.msg/PointStamped
    """
    global TRAJECTORY_DONE, TRAJECTORY_POINTS
    global PICKING_UP, PICK_UP_I, CLOSED_PATH

    cpoint = numpy.asarray([data.point.x, data.point.y])

    if len(TRAJECTORY_POINTS) == 0:
        TRAJECTORY_POINTS = numpy.array([cpoint])

    else:
        # Find distances to all points
        dists = numpy.sqrt(
            numpy.sum(
                numpy.power(TRAJECTORY_POINTS - cpoint, 2), axis = 1
            )
        )

        _dist = numpy.min(dists)
        _i = numpy.argmin(dists)

        if PICKING_UP:
            if _dist < 0.15:
                if PICK_UP_I == _i:
                    # Complete the trajectory if this is the last point
                    if (
                        not TRAJECTORY_DONE
                        and _i == len(TRAJECTORY_POINTS) - 1
                        and len(TRAJECTORY_POINTS) > 1
                    ):
                        TRAJECTORY_DONE = True
                        CLOSED_PATH = False

                    # Place it back
                    else:
                        pass

                # Delete it
                else:
                    TRAJECTORY_POINTS = numpy.delete(
                        TRAJECTORY_POINTS, PICK_UP_I, axis = 0
                    )
            else:
                # Move the point
                TRAJECTORY_POINTS[PICK_UP_I, :] = cpoint

            PICKING_UP = False
        else:
            if _dist < 0.15:
                # Close the loop
                if (
                    not TRAJECTORY_DONE
                    and _i == 0
                    and len(TRAJECTORY_POINTS) > 2
                ):
                    TRAJECTORY_DONE = True
                    CLOSED_PATH = True

                # Pick
                else:
                    PICK_UP_I = _i
                    PICKING_UP = True
            else:
                if not TRAJECTORY_DONE:
                    # Place new point in order, as when building the path,
                    # we want to have control over this
                    TRAJECTORY_POINTS = numpy.vstack(
                        (TRAJECTORY_POINTS, cpoint)
                    )

                else:
                    # Place new point in between two closest points
                    consdists = dists + numpy.roll(dists, 1)
                    closest_point_i = numpy.argmin(consdists)

                    # Now we have index of end of the gap -> we can put
                    # this point on this index
                    TRAJECTORY_POINTS = numpy.insert(
                        TRAJECTORY_POINTS, closest_point_i, cpoint, axis = 0
                    )

    # Reinterpolate the path and recompute everything
    # Only when not picking up points
    if TRAJECTORY_DONE and not PICKING_UP:
        simple_trajectory()

    NODE_HANDLE.logdebug(str(TRAJECTORY_POINTS.tolist()))

    publish_trajectory_points()


def path_callback(data):
    """Receive a path from different source.

    Arguments:
    data -- received path, defined by nav_msgs.msg/Path
    """
    global TRAJECTORY_DONE, TRAJECTORY_POINTS

    TRAJECTORY_DONE = True

    TRAJECTORY_POINTS = numpy.asarray(
        [[pose.pose.position.x, pose.pose.position.y] for pose in data.poses]
    )

    simple_trajectory()


def goal_callback(data):
    """Obtain the track pose from rViz and transform the track accordingly.

    Arguments:
    data -- received pose, defined by geometry_msgs.msg/PoseStamped

    Note: The position corresponds to the new location of the trajectory's
          first point.
    """
    global TRAJECTORY_DONE, TRAJECTORY_POINTS

    if not TRAJECTORY_DONE:
        # Trajectory not finished, so omit this.
        return

    # Obtain rotation matrix
    R = quaternion_matrix([
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w
    ])

    # Transform all trajectory points
    TRAJECTORY_POINTS = numpy.asarray([
        numpy.add(
            numpy.matmul(
                R,
                [
                    # Shift TP[0] to the origin.
                    _x - TRAJECTORY_POINTS[0][0],
                    _y - TRAJECTORY_POINTS[0][1],
                    0,
                    1
                ]
            )[:2],
            [
                data.pose.position.x,
                data.pose.position.y
            ]  # Just adding it is enough as it contains the +TP[0].
        )
        for _x, _y in TRAJECTORY_POINTS
    ])

    simple_trajectory()

    publish_trajectory_points()


def load_data(filename, delimiter = ""):
    """Load points from a file.

    Arguments:
    filename -- path to a file to load, str
    delimiter -- delimiter of the data, str, defaults to ""

    Returns:
    success

    Note:
    When `delimiter` not passed, the data are loaded using
    numpy.load(). Otherwise numpy.loadtxt() is used.
    """
    global TRAJECTORY_DONE, TRAJECTORY_POINTS, CLOSED_PATH

    if filename == "":
        return False

    if not os.path.exists(filename):
        NODE_HANDLE.logerror("File '%s' does not exist." % filename)
        return False

    if not os.access(filename, os.O_RDONLY):
        NODE_HANDLE.logerror("File '%s' is not readable." % filename)
        return False

    # Support comma
    # ROS1 does not allow to set a parameter to a single comma ','.
    # It is probably caused by the underlying yaml loader.
    # rospy.exceptions.ROSInitException: invalid command-line parameters:
    # while parsing a block node
    #  expected the node content, but found ','
    if delimiter == "\,":  # noqa: W605
        delimiter = ","

    try:
        if delimiter == "":
            # We load only first two columns.
            # Therefore, this can be used for trajectories, etc.
            TRAJECTORY_POINTS = numpy.load(filename)[:, :2]
            NODE_HANDLE.loginfo(
                "Loaded %d points from %s using 'numpy.load()'."
                % (len(TRAJECTORY_POINTS), filename)
            )
        else:
            TRAJECTORY_POINTS = numpy.loadtxt(
                filename, delimiter = delimiter
            )[:, :2]
            NODE_HANDLE.loginfo(
                "Loaded %d points (delimited by '%s') from %s "
                "using 'numpy.loadtxt()'."
                % (len(TRAJECTORY_POINTS), delimiter, filename)
            )

        TRAJECTORY_DONE = True
    except Exception as e1:
        # Try to load it as csv
        if delimiter == "":
            NODE_HANDLE.logerror(
                "Unable to load '%s' using 'numpy.load()': %s "
                "\nSet delimiter to try to load it as a csv."
                % (filename, str(e1))
            )
            return False
        else:
            try:
                with open(filename, "r") as f:
                    reader = csv.DictReader(f, delimiter = delimiter)

                    if not list_contains(["x_m", "y_m"], reader.fieldnames):
                        raise ValueError(
                            "file does not contain required fields "
                            "'x_m' and 'y_m'"
                        )

                    TRAJECTORY_POINTS = numpy.asarray([
                        [float(line["x_m"]), float(line["y_m"])]
                        for line in reader
                    ])

                    NODE_HANDLE.loginfo(
                        "Loaded %d points (delimited by '%s') from %s "
                        "using 'csv.DictReader()'."
                        % (len(TRAJECTORY_POINTS), delimiter, filename)
                    )

                    TRAJECTORY_DONE = True
            except Exception as e2:
                NODE_HANDLE.logerror(
                    "Unable to load '%s' using "
                    "'numpy.loadtxt(delimiter='%s')': %s"
                    % (filename, delimiter, str(e1))
                )
                NODE_HANDLE.logerror(
                    "Unable to load '%s' using "
                    "'csv.DictReader(delimiter='%s')': %s"
                    % (filename, delimiter, str(e2))
                )
                return False

    simple_trajectory()

    publish_trajectory_points()

    return True


######################
# Functions
######################

def start_node(args = None):
    """Start a ROS node, registers the callbacks."""
    global CLOSED_PATH, NODE_HANDLE

    if args is None:
        args = sys.argv

    Core.init(args = args)

    NODE_HANDLE = Node("simple_trajectory")


    # Register callback
    NODE_HANDLE.Subscriber("map", OccupancyGrid, map_callback)
    NODE_HANDLE.Subscriber("clicked_point", PointStamped, clicked_point)
    NODE_HANDLE.Subscriber("path", Path, path_callback)
    NODE_HANDLE.Subscriber("move_base_simple/goal", PoseStamped, goal_callback)


    # Publishers
    NODE_HANDLE.pnt_pub = NODE_HANDLE.Publisher(
        'trajectory_points', Marker, queue_size = 1, latch = True
    )
    NODE_HANDLE.path_pub = NODE_HANDLE.Publisher(
        'reference_path/path', Path, queue_size = 1, latch = True
    )
    NODE_HANDLE.pathm_pub = NODE_HANDLE.Publisher(
        'reference_path/marker', Marker, queue_size = 1, latch = True
    )
    NODE_HANDLE.map_pub = None
    NODE_HANDLE.infgc_pub = NODE_HANDLE.Publisher(
        'reference_path/gridcells', GridCells, queue_size = 1, latch = True
    )


    # Obtain parameters
    # They are not yet implemented, so we need to do this ourselves.
    if ROS_VERSION == 1:
        if NODE_HANDLE.has_param("~closed_path"):
            CLOSED_PATH = bool(NODE_HANDLE.get_param("~closed_path"))

        if NODE_HANDLE.has_param("~input_file"):
            loaded = load_data(
                str(NODE_HANDLE.get_param("~input_file")),
                str(NODE_HANDLE.get_param("~delimiter", ""))
            )

            P.input_file = (
                NODE_HANDLE.get_param("~input_file")
                if loaded else ""
            )
            P.delimiter = NODE_HANDLE.get_param("~delimiter", "")


    # Dynamic reconfigure
    # In ROS2 we have to call this before 'get_parameter'.
    # As it sets values on the ROS Parameter Server, it
    # has to be called after 'get_param' in ROS1.
    P.reconfigure(node = NODE_HANDLE)

    if ROS_VERSION == 2:
        CLOSED_PATH = NODE_HANDLE.get_parameter("closed_path").value

        loaded = load_data(
            NODE_HANDLE.get_parameter("input_file").value,
            NODE_HANDLE.get_parameter("delimiter").value
        )

        P.input_file = (
            NODE_HANDLE.get_parameter("input_file").value
            if loaded else ""
        )
        P.delimiter = NODE_HANDLE.get_parameter("delimiter").value


    Core.spin(NODE_HANDLE)

    Core.shutdown()


if __name__ == "__main__":
    INFLATE_TRAJECTORY = create_surroundings(TRAJECTORY_DISTANCE)
    INFLATE_AREA = create_surroundings(INFLATE_DISTANCE)
    start_node()
