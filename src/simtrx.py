#!/usr/bin/env python


"""
Class for simulating a trx vehicle. Receives input commands and sends position information.
"""


import rospy
import sys
import math

import trxmodel

from trucksim.msg import MocapState, PWM
from geometry_msgs.msg import Twist


class SimTrx(object):

    def __init__(self, vehicle_id,
                 mocap_topic_name, mocap_topic_type, control_topic_name, control_topic_type,
                 x=None, u=None, frequency=20):

        # Node and topic names and types.
        if u is None:
            u = [0, 0, 1]
        if x is None:
            x = [0, 0, 0]

        self.x = x  # Vehicle state [x, y, yaw].
        self.u = u  # Vehicle input [speed, angular velocity].

        self.v = 0
        self.yaw_rate = 0
        self.acceleration = 0
        self.radius = 0

        self.dt = 1. / frequency  # Update interval.

        self.last_x = self.x[:]  # Store the last state.

        self.vehicle_id = vehicle_id

        # Initialize ROS node.
        rospy.init_node(self.vehicle_id + '_simulated', anonymous=False)

        # Subscriber for receiving speed control signal.
        rospy.Subscriber(control_topic_name, control_topic_type, self._callback)

        # Publisher for publishing vehicle position and velocity.
        self.pub = rospy.Publisher(mocap_topic_name, mocap_topic_type, queue_size=10)

        # ROS update rate.
        self.update_rate = rospy.Rate(frequency)

        # Fix so that standing still at start by sending command to itself through trxvehicle.py.
        # Last command in trxvehicle.py is then set to the standstill values.
        self.pwm_start_pub = rospy.Publisher('pwm_commands', PWM, queue_size=1)

        self._initialize_standstill()

    def _initialize_standstill(self):
        for i in range(5):
            self.update_rate.sleep()
            self.pwm_start_pub.publish(self.vehicle_id, 1500, 1500, 120)

        print('Trx test vehicle initialized, id: {}'.format(self.vehicle_id))

    def _callback(self, data):
        """Method called when subscriber receives data. Updates the input. """
        self.u[0] = trxmodel.throttle_input_to_linear_velocity(data.linear.x)
        self.u[1] = trxmodel.steering_input_to_angular_velocity(data.angular.z, self.v)

    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self._move()

            self._publish_vehicle_state()

            self.update_rate.sleep()

    def _publish_vehicle_state(self):
        t = rospy.get_time()
        self.pub.publish(t, self.vehicle_id, self.x[0], self.x[1], self.x[2],
                         self.yaw_rate, self.v, self.acceleration, self.radius)

    def _move(self):
        """Moves the vehicle as a unicycle and calculates velocity, yaw rate, etc. """

        self.last_x = self.x[:]     # Save information for velocity calculation.

        # Move the vehicle according to the dynamics.
        self.x[0] = self.x[0] + self.dt * self.u[0] * math.cos(self.x[2])
        self.x[1] = self.x[1] + self.dt * self.u[0] * math.sin(self.x[2])
        self.x[2] = (self.x[2] + self.dt * self.u[1]) % (2 * math.pi)

        # Calculate yaw rate.
        if self.x[2] < self.last_x[2] - math.pi:
            yaw_difference = self.x[2] - self.last_x[2] + 2 * math.pi
        elif self.x[2] > self.last_x[2] + math.pi:
            yaw_difference = self.x[2] - self.last_x[2] - 2 * math.pi
        else:
            yaw_difference = self.x[2] - self.last_x[2]
        self.yaw_rate = yaw_difference / self.dt

        # Calculate velocity and acceleration.
        distance = math.sqrt((self.x[0] - self.last_x[0]) ** 2 + (self.x[1] - self.last_x[1]) ** 2)
        v = distance / self.dt
        self.acceleration = (v - self.v) / self.dt
        self.v = v

        # Calculate turning radius.
        try:
            self.radius = distance / yaw_difference
        except ZeroDivisionError:
            self.radius = 0

    def __str__(self):
        """Returns the name of the node as well as the vehicle position. """
        return self.vehicle_id + ': ' + super(SimTrx, self).__str__()


def main(args):
    """Creates a vehicle node and starts the simulation. The name of the vehicle is entered as an
    argument on the command line. The vehicle is initialized at the origin pointing to the left. """

    if len(args) < 1:
        print('Need to enter a vehicle ID.')
        sys.exit()

    vehicle_id = args[1]
    frequency = 20
    mocap_topic_name = 'mocap_state'
    mocap_topic_type = MocapState
    control_topic_name = vehicle_id + '/cmd_vel'
    control_topic_type = Twist

    vn = SimTrx(vehicle_id,
                mocap_topic_name, mocap_topic_type, control_topic_name, control_topic_type,
                [0, 0, math.pi], [0., 0., 1], frequency)

    vn.run()


if __name__ == '__main__':
    main(sys.argv)
