#!/usr/bin/env python

import rospy
import sys
import math
import time

from trucksim.msg import vehiclespeed
from trucksim.msg import vehicleomega
from trucksim.msg import vehicleposition

import vehicle


class VehicleNode(vehicle.Vehicle):
    """Class for simulating the movement of a vehicle. The vehicle is identified
    with the name of the ROS node. """

    def __init__(self, x = [0, 0, 0], u = [0, 0], frequency = 20, name = None,
        position_topic_name = 'vehicle_position',
        position_topic_type = vehicleposition,
        speed_topic_name = 'vehicle_speed', speed_topic_type = vehiclespeed,
        omega_topic_name = 'vehicle_omega', omega_topic_type = vehicleomega):

        # Node and topic names and types.
        DEFAULT_NODE_NAME = 'vehicle'

        # Initialize superclass.
        super(VehicleNode, self).__init__(x, u)

        # Subscriber for receiving speed control signal.
        rospy.Subscriber(speed_topic_name, speed_topic_type,
            self._speed_callback)

        # Subscriber for receiving omega control signal.
        rospy.Subscriber(omega_topic_name, omega_topic_type,
            self._omega_callback)

        # Publisher for publishing vehicle position and velocity.
        self.pub = rospy.Publisher(
            position_topic_name, position_topic_type, queue_size = 10)

        # Initialize ROS node.
        if name is not None:
            rospy.init_node(name, anonymous = False)
        else:
            rospy.init_node(DEFAULT_NODE_NAME, anonymous = True)

        # ROS update rate.
        self.r = rospy.Rate(frequency)

        # Update interval.
        self.delta_t = 1./frequency

        print('Vehicle node initialized, id: {}'.format(rospy.get_name()))


    def _speed_callback(self, data):
        """Method called when subscriber receives data. Updates the input if the
        published ID is the same as the ID of the node. """
        if data.id == rospy.get_name():
            self.u[0] = data.speed


    def _omega_callback(self, data):
        """Method called when subscriber receives data. Updates the input if the
        published ID is the same as the ID of the node. """
        if data.id == rospy.get_name():
            self.u[1] = data.omega


    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self.move(self.delta_t)

            vel = self.get_velocity()

            self.pub.publish(
                rospy.get_name(), self.x[0], self.x[1], self.x[2], vel)

            self.r.sleep()


    def __str__(self):
        """Returns the name of the node as well as the vehicle position. """
        return rospy.get_name() + ': ' + super(VehicleNode, self).__str__()


def main(args):
    """Creates a vehicle node and starts the simulation. The name of the vehicle
    is entered as an argument on the command line. If a name is not entered it is given the default name with a random unique number at the end. The
    vehicle is initialized at the origin pointing to the left. """
    if len(args) > 1:
        vn = VehicleNode([0, 0, math.pi], [0., 0.], name = args[1])
    else:
        vn = VehicleNode([0, 0, math.pi], [0., 0.])

    vn.run()


if __name__ == '__main__':
    main(sys.argv)
