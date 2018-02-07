#!/usr/bin/env python

import rospy

import vehicle_node

def main():
    vn1 = vehicle_node.VehicleNode([0, 0, 0], [0., 0.], name = 'v1')
    vn1.run()


if __name__ == '__main__':
    main()
