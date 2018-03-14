#!/usr/bin/env python

import rospy
from trucksim.msg import vehicleposition
from mocap_source_2 import *

import time
import math
import sys


class PosePublisher(object):
    """Publisher class for vehicle pose from MoCap data. """

    def __init__(self, topic_name, topic_type, mocap_vehicle_id, mocap_host_address,
                 update_freq=20, moving_average=1,
                 mocap_used=True, sim_omega=0.75, sim_radius=None, sim_center=None):

        self.mocap_vehicle_id = mocap_vehicle_id
        self.mocap_used = mocap_used

        # Vehicle pose.
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0

        self.velocity_ma = MovingAverage(moving_average)  # Moving average of velocity.

        self.pub = rospy.Publisher(topic_name, topic_type, queue_size=1)
        rospy.init_node(self.mocap_vehicle_id, anonymous=True)

        self.rate = rospy.Rate(update_freq)

        self.node_init_time = 0
        self.last_update_time = 0

        print('Pose publisher initialized for vehicle {}'.format(self.mocap_vehicle_id))

        if self.mocap_used:
            self.vehicle = MoCapVehicle(mocap_host_address, self.mocap_vehicle_id)
        else:
            self.vehicle = CircleVehicle(sim_omega, 0, sim_radius, sim_center)
            print('Using simulated vehicle. ')

    def start(self):
        """Starts the publisher. """
        print('Publisher running...')
        self.node_init_time = time.time()
        self.last_update_time = time.time()

        self._talker()

    def _talker(self):
        """Publishes the mocap or simulated data continuously to the topic. """
        while not rospy.is_shutdown():

            total_time_elapsed = time.time() - self.node_init_time

            if not self.mocap_used:
                self.vehicle.update_pose(total_time_elapsed)   # Update simulated position.

            try:
                x, y, yaw = self.vehicle.get_pose()  # Get position.
                self._update_pose(x, y, yaw)
            except:
                print('{:.1f}: Lost data.'.format(total_time_elapsed))

            # Publish vehicle pose to the topic.
            self.pub.publish(self.mocap_vehicle_id, self.x, self.y, self.theta, self.v)

            self.rate.sleep()

    def _update_pose(self, x, y, yaw):
        """Updates the vehicle pose. Calculates a new velocity by using the old position."""
        elapsed_time = time.time() - self.last_update_time
        v = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2) / elapsed_time
        self.last_update_time = time.time()

        self.v = self.velocity_ma.new_ma(v)
        self.x = x
        self.y = y
        self.theta = yaw


class MoCapVehicle(object):
    """Class for getting data from Mocap. """
    def __init__(self, host_address, vehicle_id):
        self.mocap = Mocap(host=host_address, info=1)
        self.mocap_body = self.mocap.get_id_from_name(vehicle_id)

    def get_pose(self):
        """Returns the vehicle state. """
        state = self.mocap.get_body(self.mocap_body)
        x = state['x']
        y = state['y']
        yaw = state['yaw']

        return x, y, yaw*math.pi/180


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


class CircleVehicle(object):
    """Class for a simulated vehicle driving in a circle. Does not accept inputs, only supplies
    its position. """
    def __init__(self, omega=0.75, alpha0=0., radius=None, offset=None):

        if radius is None:
            radius = [1., 1.]
        if offset is None:
            offset = [0., 0.]

        self.omega = omega          # Angular velocity.
        self.alpha0 = alpha0        # Angle to initial position on path.
        self.xradius = radius[0]    # x-radius of ellipse path.
        self.yradius = radius[1]
        self.xc = offset[0]         # x-coordinate of ellipse path center.
        self.yc = offset[1]

        self.x = self.xc + self.xradius*math.cos(self.alpha0)
        self.y = self.yc + self.yradius*math.sin(self.alpha0)
        self.theta = self.alpha0 + math.pi / 2   # Angle of truck.

    def update_pose(self, time_elapsed):
        """Update the position of the simulated truck. """
        alpha = self.alpha0 + time_elapsed * self.omega

        self.x = self.xc + self.xradius*math.cos(alpha)
        self.y = self.yc + self.yradius*math.sin(alpha)
        self.theta = (alpha + math.pi / 2) % (2 * math.pi)

    def get_pose(self):
        """Return the position and orientation of the simulated truck. """
        return self.x, self.y, self.theta


def main(args):
    mocap_used = True           # True if using Mocap, False if using simulation.

    if len(args) > 1:
        mocap_vehicle_id = args[1]
    else:
        print('Need to enter the ID of the vehicle. ')
        sys.exit()

    try:
        if int(args[2]) == 0:   # If the second argument is zero a simulated vehicle will be used.
            mocap_used = False
    except:
        pass

    freq = 10               # Publishing frequency in Hz.
    moving_average_num = 1  # How many values to use for moving average calculation of velocity.

    # Publisher node info.
    topic_name = 'vehicle_position' # Name of ROS topic.
    topic_type = vehicleposition    # Type of ROS topic.

    mocap_address = '192.168.1.10'  # IP-address of the MoCap computer.

    # Data if using a simulated vehicle for testing.
    sim_omega = 0.75         # Angular velocity of vehicle.
    sim_radius = [1.3, 1.7]   # Radii of ellipse path.
    sim_center = [0.3, -1.3]  # Center of ellipse path.

    # Create and run the publisher.
    publisher = PosePublisher(topic_name, topic_type, mocap_vehicle_id, mocap_address,
                              update_freq=freq, moving_average=moving_average_num, mocap_used=mocap_used,
                              sim_omega=sim_omega, sim_radius=sim_radius, sim_center=sim_center)

    publisher.start()   # Start the publisher.


if __name__ == '__main__':
    main(sys.argv)
