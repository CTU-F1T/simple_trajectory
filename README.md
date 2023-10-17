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
- `autopsy>0.9.4`


## Usage

```
rosrun simple_trajectory node.py
rosrun simple_trajectory node.py _input_file:=FILENAME _delimiter:=DELIMITER _closed_path:=TRUE_OR_FALSE
```

or

```
ros2 run simple_trajectory run
ros2 run simple_trajectory run --ros-args -p input_file:=FILENAME -p delimiter:=DELIMITER -p closed_path:=TRUE_OR_FALSE
```


## Parameters

- Path parameters
  - `trajectory_inflate` (int): Radius of the path inflation in cells.
  - `map_inflate` (int): Radius of map inflation in cells.
  - `publish_cropped_map` (bool): When True, inflated and cropped map is published.
  - `reload_map` (bool): When True, internal map is updated from the map topic.
- Loading parameters
  - `closed_path` (bool): When True, interpret the loaded file / received points as a closed path.
  - `input_file` (str): When given, load points from the file. When `delimiter` is empty, load it as npz, otherwise load it as a csv.
  - `delimiter` (str): Delimiter used in the csv file.

_Note: Delimiter cannot be a comma, as ROS complains about it._
