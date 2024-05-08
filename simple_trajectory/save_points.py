#!/usr/bin/env python2
# save_points.py
"""Helper script for saving the trajectory points from the simple_trajectory.

Expecting one argument with filename. Points are saved as a csv.
"""
######################
# Imports & Globals
######################

import argparse

# ROS wrapper
from autopsy.node import Node
from autopsy.core import Core

# ROS Messages
from visualization_msgs.msg import Marker


PARSER = argparse.ArgumentParser(
    prog = "save_points",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description = """
Utility for saving trajectory points from simple_trajectory.
    """,  # noqa: E501
)

PARSER.add_argument(
    "output",

    nargs = "?",
    help = "Output file, %%s",
    metavar = "OUTPUT_FILE",
)


######################
# SavePointsNode
######################

class SavePointsNode(Node):
    """Helper node for saving points from simple_trajectory."""

    def __init__(self, *args, **kwargs):
        """Initialize the node."""
        super(SavePointsNode, self).__init__(*args, **kwargs)


######################
# Main
######################

if __name__ == "__main__":
    args, other_args = PARSER.parse_known_args()

    if args.output is None:
        print ("Expected one argument with output file name.")
        exit(1)

    try:
        open(args.output, "w")
    except Exception as e:
        print ("Unable to write into '%s': %s" % (args.output, e))
        exit(2)

    Core.init(args = other_args)

    n = SavePointsNode("simple_trajectory_save_points")

    n.loginfo("Waiting for message on topic '/trajectory_points'...")

    msg = n.wait_for_message("/trajectory_points", Marker)

    n.loginfo("Received message.")

    with open(args.output, "w") as outfile:
        for point in msg.points:
            outfile.write("%.9f,%.9f\n" % (point.x, point.y))

    n.loginfo("Wrote %d points into '%s'." % (len(msg.points), args.output))

    Core.shutdown()
