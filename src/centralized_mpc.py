#!/usr/bin/env python

"""
Centralized MPC controller.
"""

import rospy
import sys
import numpy

import speed_profile
import path
import frenetpid
import centralized_mpc_solver

from trucksim.msg import MocapState, PWM


class CentralizedMPC(object):

    def __init__(self, position_topic_name, control_topic_name, vehicle_ids,
                 vehicle_path, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, k_p, k_i, k_d,
                 xmin=None, xmax=None, umin=None, umax=None, vopt=None):

        self.dt = delta_t
        self.vehicle_ids = vehicle_ids

        self.running = False

        rospy.init_node('centralized_mpcs', anonymous=True)

        # Publisher for controlling vehicles.
        self.pwm_publisher = rospy.Publisher(control_topic_name, PWM, queue_size=1)

        # Subscriber for vehicle positions.
        rospy.Subscriber(position_topic_name, MocapState, self._position_callback)

        self.rate = rospy.Rate(1./self.dt)

        # Optimal speed profile in space. If none given, optimal speed is 1 m/s everywhere.
        if vopt is None:
            vopt = speed_profile.Speed([1], [1])
        self.vopt = vopt

        self.mpc = centralized_mpc_solver.MPC(len(self.vehicle_ids), Ad, Bd, self.dt, horizon, zeta,
                                              Q, R, truck_length, safety_distance, timegap,
                                              xmin, xmax, umin, umax)

        self.frenets = dict()
        self.path_positions = dict()
        for vehicle_id in self.vehicle_ids:
            self.frenets[vehicle_id] = frenetpid.FrenetPID(vehicle_path, k_p, k_i, k_d)
            self.path_positions[vehicle_id] = path.PathPosition(vehicle_path)

    def control(self):
        if not self.running:
            return

    def _position_callback(self, data):
        """Callback for subscriber subscribing to vehicle positions. """
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def run(self):
        """Runs the controller. Used when global GUI. """
        while not rospy.is_shutdown():
            self.control()
            self.rate.sleep()

        self.stop()


def main(args):

    if len(args) > 1:
        vehicle_ids = args[1:]
    else:
        print('Need to enter at least one vehicle ID. ')
        sys.exit()

    # Topic name for subscribing to truck positions.
    position_topic_name = 'mocap_state'

    # Topic name for publishing vehicle commands.
    control_topic_name = 'pwm_commands'

    # PID parameters for path tracking.
    k_p = 0.5
    k_i = 0
    k_d = 3

    horizon = 5
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.90
    s0 = 0.
    v0 = 0.
    Q_v = 1  # Part of Q matrix for velocity tracking.
    Q_s = 0.5  # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)  # State tracking.
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    velocity_min = 0.
    velocity_max = 2.
    position_min = -100000.
    position_max = 1000000.
    acceleration_min = -0.5
    acceleration_max = 0.5
    truck_length = 0.2
    safety_distance = 0.1
    timegap = 1.

    x0 = numpy.array([s0, v0])
    xmin = numpy.array([velocity_min, position_min])
    xmax = numpy.array([velocity_max, position_max])
    umin = numpy.array([acceleration_min])
    umax = numpy.array([acceleration_max])

    # Reference speed profile.
    opt_v_pts = 1000  # How many points.
    opt_v_max = 1.2
    opt_v_min = 0.8
    opt_v_period_length = 60  # Period in meters.
    # vopt = speed_profile.Speed([1], [1])
    vopt = speed_profile.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

    # Controller reference path.
    x_radius = 1.4
    y_radius = 1.2
    center = [0.2, -y_radius / 2]
    pts = 400

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    mpc = CentralizedMPC(position_topic_name, control_topic_name, vehicle_ids,
                 pt, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, k_p, k_i, k_d, xmin=xmin, xmax=xmax, umin=umin,
                 umax=umax, vopt=vopt)


if __name__ == '__main__':
    main(sys.argv)