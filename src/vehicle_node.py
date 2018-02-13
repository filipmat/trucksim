#!/usr/bin/env python

import rospy
import sys
import math

from trucksim.msg import vehiclespeed
from trucksim.msg import vehicleomega
from trucksim.msg import vehicleposition

import vehicle


class VehicleNode(vehicle.Vehicle):
    """Class for simulating the movement of a vehicle. The vehicle is identified
    with the name of the ROS node. """

    def __init__(self, x = [0, 0, 0], u = [0, 0], frequency = 20,
        name = None):

        # Node and topic names and types.
        DEFAULT_NAME = 'vehicle'
        SPEED_TOPIC_NAME = 'vehicle_speed'
        OMEGA_TOPIC_NAME = 'vehicle_omega'
        POSITION_TOPIC_NAME = 'vehicle_position'
        SPEED_TOPIC_TYPE = vehiclespeed
        OMEGA_TOPIC_TYPE = vehicleomega
        POSITION_TOPIC_TYPE = vehicleposition

        # Initialize superclass.
        super(VehicleNode, self).__init__(x, u)

        # Subscriber for receiving control inputs.
        rospy.Subscriber(SPEED_TOPIC_NAME, SPEED_TOPIC_TYPE,
            self._speed_callback)
        rospy.Subscriber(OMEGA_TOPIC_NAME, OMEGA_TOPIC_TYPE,
            self._omega_callback)

        # Publisher for publishing vehicle position.
        self.pub = rospy.Publisher(
            POSITION_TOPIC_NAME, POSITION_TOPIC_TYPE, queue_size = 10)

        # Initialize ROS node.
        if name is not None:
            rospy.init_node(name, anonymous = False)
        else:
            rospy.init_node(DEFAULT_NAME, anonymous = True)

        # ROS update rate.
        self.r = rospy.Rate(frequency)

        # Update interval.
        self.delta_t = 1./frequency

        print('Vehicle node initialized, id: {}'.format(rospy.get_name()))


    def _speed_callback(self, data):
        """Method called when subscriber receives data. Updates the input if the
        published ID is the same as the ID of the node. """
        if data.id == rospy.get_name():
            self.set_speed(data.speed)


    def _omega_callback(self, data):
        """Method called when subscriber receives data. Updates the input if the
        published ID is the same as the ID of the node. """
        if data.id == rospy.get_name():
            self.set_omega(data.omega)


    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self.move(self.delta_t)

            vel = self._get_velocity()

            self.pub.publish(
                rospy.get_name(), self.x[0], self.x[1], self.x[2], vel)

            self.r.sleep()


    def __str__(self):
        """Returns the name of the node as well as the vehicle position. """
        return rospy.get_name() + ': ' + super(VehicleNode, self).__str__()


def main(args):
    if len(args) > 1:
        vn = VehicleNode([0, 0, math.pi], [0., 0.], name = args[1])
    else:
        vn = VehicleNode([0, 0, math.pi], [0., 0.])

    vn.run()


if __name__ == '__main__':
    main(sys.argv)
