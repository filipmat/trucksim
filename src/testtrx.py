#!/usr/bin/env python

import rospy
import sys
import math

from trucksim.msg import MocapState
from trucksim.msg import PWM


class TestTrxNode(object):
    """Class for simulating the movement of a vehicle. The vehicle is identified
    with the name of the ROS node. """

    def __init__(self, vehicle_id,
                 mocap_topic_name, mocap_topic_type,
                 control_topic_name, control_topic_type,
                 x=None, u=None, frequency=20):

        # Node and topic names and types.
        if u is None:
            u = [0, 0, 1]
        if x is None:
            x = [0, 0, 0]

        self.x = x  # Vehicle state.
        self.u = u  # Vehicle input.

        # Variables used for velocity calculation.
        self.delta_t = 0  # Stores the update interval.
        self.last_x = self.x[:]  # Stores the last state.
        self.v = 0  # Vehicle velocity.

        self.last_x = self.x

        self.vehicle_id = vehicle_id

        # Subscriber for receiving speed control signal.
        rospy.Subscriber(control_topic_name, control_topic_type, self._callback)

        # Publisher for publishing vehicle position and velocity.
        self.pub = rospy.Publisher(mocap_topic_name, mocap_topic_type, queue_size=10)

        # Initialize ROS node.
        rospy.init_node(self.vehicle_id, anonymous=True)

        # ROS update rate.
        self.r = rospy.Rate(frequency)

        # Update interval.
        self.delta_t = 1. / frequency

        print('Vehicle node initialized, id: {}'.format(rospy.get_name()))

    def _callback(self, data):
        """Method called when subscriber receives data. Updates the input if the
        published ID is the same as the ID of the node. """
        self.u[0] = data.velocity
        self.u[1] = data.angle
        self.u[2] = data.gear

    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self.move(self.delta_t)

            vel = self.get_velocity()

            yaw_rate = self.get_yaw_rate()

            self.pub.publish(self.vehicle_id, self.x[0], self.x[1], self.x[2], yaw_rate, vel, 0, 0)

            self.r.sleep()

    def move(self, delta_t):
        """Moves the vehicle. The vehicle moves as a unicycle. """

        # Save information for velocity calculation.
        self.last_x = self.x[:]
        self.delta_t = delta_t

        # Move the vehicle according to the dynamics.
        self.x[0] = self.x[0] + delta_t * self.u[0] * math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t * self.u[0] * math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t * self.u[1]) % (2 * math.pi)

    def get_velocity(self):
        """Returns the velocity of the vehicle. The velocity is calculated
        using the distance traveled from the last position and the elapsed time
        from the last movement. The elapsed time is the update interval. """
        distance = math.sqrt((self.x[0] - self.last_x[0]) ** 2 + (self.x[1] - self.last_x[1]) ** 2)

        # Calculate the velocity.
        self.v = distance / self.delta_t

        return self.v

    def get_yaw_rate(self):
        if self.x[2] < self.last_x[2] - math.pi:
            theta_diff = self.x[2] - self.last_x[2] + 2 * math.pi
        elif self.x[2] > self.last_x[2] + math.pi:
            theta_diff = self.x[2] - self.last_x[2] - 2 * math.pi
        else:
            theta_diff = self.x[2] - self.last_x[2]
        yaw_rate = theta_diff / self.delta_t

        return yaw_rate

    def __str__(self):
        """Returns the name of the node as well as the vehicle position. """
        return self.vehicle_id + ': ' + super(TestTrxNode, self).__str__()


def main(args):
    """Creates a vehicle node and starts the simulation. The name of the vehicle
    is entered as an argument on the command line. The
    vehicle is initialized at the origin pointing to the left. """

    if len(args) < 1:
        print('Need to enter a vehicle ID.')
        sys.exit()

    vehicle_id = args[1]
    frequency = 20
    mocap_topic_name = 'mocap_state'
    mocap_topic_type = MocapState
    control_topic_name = 'pwm_commands'
    control_topic_type = PWM

    vn = TestTrxNode(args[1],
                     mocap_topic_name, mocap_topic_type,
                     control_topic_name, control_topic_type,
                     [0, 0, math.pi], [0., 0., 1], frequency)

    vn.run()


if __name__ == '__main__':
    main(sys.argv)
