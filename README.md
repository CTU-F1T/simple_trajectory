# simple trajectory
This node is used to create a custom path using 'Publish Point' feature that is available in rViz.

```
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
```

This node is used to create a custom path using 'Publish Point' feature that is available in rViz. When a global map is provided, it is used to compute surrounding area of the path to define all valid positions.

Using 'Publish Point' you can place path via-points. They are published on topic 'trajectory_points'.

First placed point has purple color, other points are red. When at least three points are placed, placing a new point on the first (purple) one sends a signal to close the path, interpolate it using CubicSpline and publish it. As a note, placed points are interpolated in placing order.

Placing a point near another red point (or purple point in case that there are only less then three points) colors this point yellow. Now, there are three options according to where you place the next point:
- placing it on the yellow point makes it red again as nothing has happened,
- placing it somewhere else moves the yellow point to this location; ordering of the points is not changed,
- placing it on another point deletes the yellow point.

Additionally, placing a point somewhere in the environment leads to finding two closest via-points on the path, and this new point is placed in-between of them.


## Requirements

- `python-numpy`
- `python-scipy`


## Usage

```
roslaunch simple_trajectory start.launch
```
