#!/usr/bin/env python

# File for testing stuff.
# Creates a vehicle node at the origin and starts the simulated movement.
# The name of the node is entered as an argument in the command line. 

import rospy
import sys
import math

import vehicle_node

def main(args):
    if len(args) > 1:
        vn = vehicle_node.VehicleNode([0, 0, math.pi], [0., 0.], name = args[1])
    else:
        vn = vehicle_node.VehicleNode([0, 0, math.pi], [0., 0.])

    vn.run()


if __name__ == '__main__':
    main(sys.argv)
