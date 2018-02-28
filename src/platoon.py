#!/usr/bin/env python

import rospy
import sys
import time

from trucksim.msg import vehicleposition
from trucksim.msg import vehiclespeed
from trucksim.msg import vehicleomega
import path
import frenetpid


class PID(object):
    def __init__(self, k_p=0., k_i=0., k_d=0.):
        # PID parameters.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.e = 0  # Current error.
        self.old_e = 0  # Previous error.
        self.sum_e = 0  # Accumulated error.

    def get_u(self, e):
        """Calculate the control input omega. """
        e_p = e - self.old_e

        self.old_e = self.e
        self.e = e

        self.sum_e += e

        # PID controller.
        u = - self.k_p * e - self.k_d * e_p - self.k_i * self.sum_e

        return u

    def set_pid(self, kp=None, ki=None, kd=None):
        """Sets the PID parameters. """
        if kp is not None:
            self.k_p = kp
        if ki is not None:
            self.k_i = ki
        if kd is not None:
            self.k_d = kd

        self.reset_sum()

    def get_pid(self):
        """Returns the PID parameters. """
        return self.k_p, self.k_i, self.k_d

    def reset_sum(self):
        """Resets the sum for I part in PID controller. """
        self.sum_e = 0

    def get_error(self):
        """Returns the latest y error. """
        return self.e


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, position_topic_type, position_topic_name,
                 speed_topic_type, speed_topic_name,
                 omega_topic_type, omega_topic_name, vehicle_ids,
                 node_name='controller', v=1., k_p=0., k_i=0., k_d=0., rate=20,
                 distance_offset=0.4, e_ref=0.5, k_pv=0., k_iv=0., k_dv=0.,
                 vmax = 40):

        self.v = v  # Desired velocity of the vehicle.
        self.vehicle_ids = ['/' + v_id for v_id in vehicle_ids]  # IDs.
        self.running = False  # If controller is running or not.
        self.rate = rate
        self.distance_offset = distance_offset
        self.e_ref = e_ref

        self.k_pv = k_pv
        self.k_iv = k_iv
        self.k_dv = k_dv

        self.vmin = 0
        self.vmax = vmax

        self.headstart_samples = 10

        # Setup ROS node.
        rospy.init_node(node_name, anonymous=True)

        # Publisher for controlling vehicle.
        self.pub_speed = rospy.Publisher(speed_topic_name, speed_topic_type,
                                         queue_size=1)

        self.pub_omega = rospy.Publisher(omega_topic_name, omega_topic_type,
                                         queue_size=1)

        # Subscriber for vehicle positions.
        rospy.Subscriber(
            position_topic_name, position_topic_type, self._callback)

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controllers and dict of last positions.
        self.frenets = dict()
        self.pids = dict()
        self.positions = dict()
        for i, vehicle_id in enumerate(self.vehicle_ids):
            self.frenets[vehicle_id] = frenetpid.FrenetPID(
                self.pt, k_p, k_i, k_d)

            self.positions[vehicle_id] = [0, 0, 0, 0]

            if i > 0:
                self.pids[vehicle_id] = PID(self.k_pv, self.k_iv, self.k_dv)

        print('\nController initialized. Vehicles {}.\n'.format(
            self.vehicle_ids))

    def _callback(self, data):
        """Called when the subscriber receives data. """
        # Retrieve data.
        vehicle_id = data.id
        x = data.x
        y = data.y
        theta = data.theta
        vel = data.v

        self.positions[vehicle_id] = [x, y, theta, vel]

    def _control(self):
        """Perform control actions from received data. Sends new values to
        truck. """
        j = 0
        while not rospy.is_shutdown():
            if self.running:
                # Track the path for each vehicle.
                for i, vehicle_id in enumerate(self.vehicle_ids):
                    omega = self._get_omega(vehicle_id)
                    self.pub_omega.publish(vehicle_id, omega)
                    if i == 0:
                        print('{:.1f}'.format(
                            self.frenets[vehicle_id].get_y_error()))

                # At the start of operation: start each vehicle with a constant
                # speed one after another, delayed with a set amount of samples.
                if j < self.headstart_samples * len(self.vehicle_ids):
                    index = j / self.headstart_samples
                    vehicle_id = self.vehicle_ids[index]

                    self.pub_speed.publish(vehicle_id, self.v)

                # Normal operation: only control the speeds of the follower
                # vehicles. The velocities are obtained from the PIDs.
                else:
                    for i, vehicle_id in enumerate(self.vehicle_ids[1:]):
                        vel = self._get_vel(vehicle_id)
                        self.pub_speed.publish(vehicle_id, vel)

                time.sleep(1. / self.rate)

            j += 1

    def _get_omega(self, vehicle_id):
        """Returns the control input omega for the specified vehicle. """
        pos = self.positions[vehicle_id]
        omega = self.frenets[vehicle_id].get_omega(
            pos[0], pos[1], pos[2], pos[3])

        return omega

    def _get_vel(self, vehicle_id):
        """Returns the velocity control signal for the given vehicle. Calculates
        the distance error to the vehicle in front and gets the velocity from
        the PID controller. """
        id1 = self.vehicle_ids[self.vehicle_ids.index(vehicle_id) - 1]
        pos1 = self.positions[id1]
        pos2 = self.positions[vehicle_id]

        dist = self.pt.get_distance([pos1[0], pos1[1]], [pos2[0], pos2[1]])
        e = self.e_ref - (dist - self.distance_offset)

        vel = self.pids[vehicle_id].get_u(e)

        if vel < self.vmin:
            vel = self.vmin
        if vel > self.vmax:
            vel = self.vmax

        return vel

    def stop(self):
        """Stops/pauses the controller. """
        for vehicle_id in self.vehicle_ids:
            self.pub_speed.publish(vehicle_id, 0)

        if self.running:
            self.running = False
            print('Controller stopped.\n')

    def start(self):
        """Starts the controller. """
        if len(self.pt.path) == 0:
            print('Error: no reference path to follow.')
            return
        if not self.running:
            self.running = True
            print('Controller started.')

    def set_reference_path(self, radius, center=None, pts=400):
        """Sets a new reference ellipse path. """
        if center is None:
            center = [0, 0]

        self.pt.gen_circle_path(radius, pts, center)

    def run(self):
        """Runs the controller. """
        self.start()
        self._control()
        self.stop()

    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters. """
        for vehicle_id in self.vehicle_ids:
            self.frenets[vehicle_id].set_pid(kp, ki, kd)

    def set_speed(self, v):
        """Sets the vehicle speed. """
        self.v = v


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_ids = args[1:]

    # Name of ROS node.
    node_name = 'controller'

    # Topic information for subscribing to truck positions.
    position_topic_name = 'vehicle_position'
    position_topic_type = vehicleposition

    # Topic information for publishing vehicle commands.
    speed_topic_name = 'vehicle_speed'
    speed_topic_type = vehiclespeed
    omega_topic_name = 'vehicle_omega'
    omega_topic_type = vehicleomega

    scale = 1

    # Data for controller reference path.
    x_radius = 1.7*scale
    y_radius = 1.2*scale
    center = [0, -y_radius]

    # Constant velocity of lead vehicle.
    v = 1*scale

    # PID parameters.
    # 0.0001
    k_p = 0.00003
    k_i = -0.02*0
    k_d = 0.020

    if scale == 1:
        k_p = 0.5
        k_i = 0
        k_d = 3

    k_pv = 2
    k_iv = 0.1
    k_dv = 1
    e_ref = 10
    distance_offset = 10

    rate = 20
    vmax = 200

    # Initialize controller.
    controller = Controller(
        position_topic_type, position_topic_name,
        speed_topic_type, speed_topic_name,
        omega_topic_type, omega_topic_name, vehicle_ids, node_name,
        v=v, k_p=k_p, k_i=k_i, k_d=k_d,
        k_pv=k_pv, k_iv=k_iv, k_dv=k_dv,
        e_ref=e_ref, distance_offset=distance_offset, rate=rate, vmax=vmax)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center, pts=400)

    # Start controller.
    controller.run()


if __name__ == '__main__':
    main(sys.argv)
