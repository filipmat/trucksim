#!/usr/bin/env python

"""
Usage: rosrun trucksim measurement.py [vehicle_id]
"""

import sys
import time
import math

import rospy
from trucksim.msg import vehicleposition


class Measurement(object):
    """Class for performing measurements for one vehicle. The measurements are the velocity,
    angular velocity, turning radius, and acceleration. """

    def __init__(self, topic_name, topic_type, vehicle_id, moving_average_amount=1,
                 print_frequency=20):

        self.vehicle_id = vehicle_id

        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0

        self.last_update_time = time.time()
        self.init_time = time.time()
        self.first_callback = True

        self.speed_ma = MovingAverage(moving_average_amount)
        self.omega_ma = MovingAverage(moving_average_amount)
        self.radius_ma = MovingAverage(moving_average_amount)
        self.acceleration_ma = MovingAverage(moving_average_amount)

        rospy.Subscriber(topic_name, topic_type, self.callback)
        rospy.init_node('measurement', anonymous=True)

        self.rate = rospy.Rate(print_frequency)


    def callback(self, data):
        """Called when subscriber receives data. Calculate speed, angular velocity, turning radius,
        and acceleration, and add them all to their respective moving average calculators. """
        if data.id != self.vehicle_id:
            return

        x = data.x
        y = data.y
        theta = data.theta
        v = data.v

        elapsed_time = time.time() - self.last_update_time
        self.last_update_time = time.time()

        # Don't try to calculate anything dumb the first time data is received.
        if self.first_callback:
            speed = 0
            omega = 0
            radius = 0
            acceleration = 0
            self.first_callback = False
        else:
            # Calculate velocity.
            speed = v

            # Calculate acceleration.
            acceleration = (v - self.v) / elapsed_time

            # Calculate angular velocity.
            # Fix angle when going from 2pi to 0, -pi to +pi etc. Assumes that has happened when
            # the angle difference is larger than pi. Decrease or increase by 2pi accordingly.
            if theta < self.theta - math.pi:
                theta_diff = theta - self.theta + 2*math.pi
            elif theta > self.theta + math.pi:
                theta_diff = theta - self.theta - 2*math.pi
            else:
                theta_diff = theta - self.theta
            omega = theta_diff / elapsed_time

            # Calculate radius. Calculated as one over the curvature.
            try:
                radius = math.sqrt((x - self.x)**2 + (y - self.y)**2) / theta_diff
            except ZeroDivisionError:
                radius = 0

        # Update old values that will be used in the next calculations.
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v

        # Update moving averages.
        self.speed_ma.new_ma(speed)
        self.omega_ma.new_ma(omega)
        self.radius_ma.new_ma(radius)
        self.acceleration_ma.new_ma(acceleration)


    def run(self):
        """Runs as long as ROS is running. Prints the measurements in the terminal at the specified
        frequency. """
        while not rospy.is_shutdown():
            print('t = {:5.1f} | v = {:5.2f} | w = {:5.2f} | r = {:5.2f} | a = {:5.2f}'.format(
                time.time() - self.init_time, self.speed_ma.get_ma(), self.omega_ma.get_ma(),
                self.radius_ma.get_ma(), self.acceleration_ma.get_ma()))

            self.rate.sleep()


class MovingAverage(object):
    """Class for calculating a moving average. """
    def __init__(self, N):
        """N: how many numbers that should be averaged. """
        self.N = N

        self.values = [None for i in range(N)]  # List containing the values.
        self.i = 0      # Index keeping track of where to enter the new value.
        self.ma = 0     # The moving average.

    def new_ma(self, value):
        """Inserts the value and returns the new moving average. The oldest value in the list is
        replaced with the new value. """
        value = float(value)

        # If the list is not yet full (start of operation), add the new value at the next position
        # and update the moving average.
        if self.i < self.N:
            self.values[self.i] = value
            self.ma = self.ma*self.i/(self.i + 1) +  value/(self.i + 1)
            self.i += 1
        # If the list is full (normal operation) update the moving average and replace the oldest
        # value in the list with the new value.
        else:
            index = self.i % self.N
            self.ma = self.ma - self.values[index]/self.N + value/self.N
            self.values[index] = value
            self.i += 1

        return self.ma

    def get_ma(self):
        """Returns the moving average without inserting a new value. """
        return self.ma


def main(args):
    if len(args) < 2:
        print('Need to enter a vehicle ID')
        sys.exit()
    else:
        vehicle_id = args[1]

    topic_type = vehicleposition        # ROS topic type.
    topic_name = 'vehicle_position'     # ROS topic name.
    moving_average = 10                 # How many values are used for moving average.
    print_frequency = 5                 # The frequency that information is printed in terminal at.

    measurement = Measurement(topic_name, topic_type, vehicle_id, moving_average,
                              print_frequency=print_frequency)

    measurement.run()


if __name__ == '__main__':
    main(sys.argv)