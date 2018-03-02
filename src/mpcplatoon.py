#!/usr/bin/env python

import rospy
import sys
import time
import numpy

from trucksim.msg import vehicleposition
from trucksim.msg import vehiclespeed
from trucksim.msg import vehicleomega
import path
import frenetpid
import truckmpc
import speed


# TEST
def print_numpy(a):
    str = '['
    for v in a.flatten():
        str += ' {:.1f}'.format(v)
    str += ' ]'

    print(str)
# TEST


class PathPosition(object):
    """Class for keeping track of absolute vehicle position on the path.
    The position increases with each lap, i.e. does not reset to zero. """
    def __init__(self, pt):
        self.pt = pt
        self.position = 0
        self.path_length = self.pt.get_path_length()
        self.zero_passes = 0
        # Allow backwards travel distance less than a fraction of path length.
        self.backwards_fraction = 1./8

    def update_position(self, xy):
        """Updates the position on the path. """
        pos = self.pt.get_position_on_path(xy)

        if (self.position >
                self.zero_passes*self.path_length + pos + self.path_length/8):
            self.zero_passes += 1

        self.position = self.zero_passes*self.path_length + pos

    def get_position(self):
        """Returns the position on the path. """
        return self.position

    def __str__(self):
        return '{:.2f}'.format(self.position)


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, position_topic_type, position_topic_name,
                 speed_topic_type, speed_topic_name,
                 omega_topic_type, omega_topic_name, vehicle_ids, path,
                 Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 node_name='controller', v=1., k_p=0., k_i=0., k_d=0., rate=20,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, QN=None):

        self.v = v  # Desired velocity of the vehicle.
        self.vehicle_ids = ['/' + v_id for v_id in vehicle_ids]  # IDs.
        self.running = False  # If controller is running or not.
        self.rate = 1/delta_t

        self.dt = delta_t
        self.headstart_samples = int(2./self.dt)

        self.vopt = speed.Speed()

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
        self.pt = path

        # Create frenet controllers and dict of last positions.
        self.frenets = dict()
        self.positions = dict()
        self.mpcs = dict()
        self.path_positions = dict()
        for i, vehicle_id in enumerate(self.vehicle_ids):
            self.frenets[vehicle_id] = frenetpid.FrenetPID(
                self.pt, k_p, k_i, k_d)

            self.positions[vehicle_id] = [0, 0, 0, 0]

            self.mpcs[vehicle_id] = truckmpc.TruckMPC(Ad, Bd, delta_t, horizon,
                                                      zeta, Q, R, truck_length,
                                                      safety_distance, timegap,
                                                      xmin=xmin, xmax=xmax,
                                                      umin=umin, umax=umax,
                                                      x0=x0, QN=QN,
                                                      id=vehicle_id)
            if i == 0:
                self.mpcs[vehicle_id].set_leader(True)

            self.path_positions[vehicle_id] = PathPosition(self.pt)

        print('\nController initialized. Vehicles {}.\n'.format(
            self.vehicle_ids))

    def set_vopt(self, vopt):
        """Sets the optimal speed profile. """
        self.vopt = vopt

    def _callback(self, data):
        """Called when the subscriber receives data. Store vehicle global
        position, orientation and velocity and position on path. """
        vehicle_id = data.id
        x = data.x
        y = data.y
        theta = data.theta
        vel = data.v

        self.positions[vehicle_id] = [x, y, theta, vel]
        try:
            self.path_positions[vehicle_id].update_position([x, y])
        except:
            pass

    def _control(self):
        """Perform control actions from received data. Sends new values to
        truck. """
        for j in range(len(self.vehicle_ids)):

            if self.running:

                for i in range(self.headstart_samples):

                    for vehicle_id in self.vehicle_ids:
                        omega = self._get_omega(vehicle_id)
                        self.pub_omega.publish(vehicle_id, omega)

                    if i < self.headstart_samples - 3:
                        acc = 1.5*self.v/(self.headstart_samples*self.dt)
                    else:
                        acc = 0

                    self.pub_speed.publish(self.vehicle_ids[j], acc)

                    time.sleep(self.dt)
                    print('{}/{}: Starting {}'.format(
                        i, self.headstart_samples - 1, self.vehicle_ids[j]))

        while not rospy.is_shutdown():
            if self.running:

                for vehicle_id in self.vehicle_ids:
                    omega = self._get_omega(vehicle_id)
                    self.pub_omega.publish(vehicle_id, omega)

                    acc = self._get_acc(vehicle_id)
                    self.pub_speed.publish(vehicle_id, acc)

                time.sleep(self.dt)

    def _get_omega(self, vehicle_id):
        """Returns the control input omega for the specified vehicle. """
        pos = self.positions[vehicle_id]
        omega = self.frenets[vehicle_id].get_omega(
            pos[0], pos[1], pos[2], pos[3])

        return omega

    def _get_acc(self, vehicle_id):
        """Returns the velocity control signal for the given vehicle. Calculates
        the distance error to the vehicle in front and gets the velocity from
        the PID controller. """
        v = self.positions[vehicle_id][3]
        path_pos = self.path_positions[vehicle_id].get_position()
        self.mpcs[vehicle_id].set_new_x0(numpy.array([v, path_pos]))

        if self.vehicle_ids.index(vehicle_id) == 0:

            self.mpcs[vehicle_id].compute_optimal_trajectories(self.vopt)
            acc = self.mpcs[vehicle_id].get_instantaneous_acceleration()

            # TEST
            print('id = {}, v = {:.1f}, a = {:.1f}'.format(vehicle_id, v, acc))
            m = self.mpcs[vehicle_id]
            #print_numpy(m.pos_ref)
            #print_numpy(m.get_assumed_state())
            print_numpy(m.get_input_trajectory())
            # TEST

            return acc

        else:

            id_prec = self.vehicle_ids[self.vehicle_ids.index(vehicle_id) - 1]
            preceding_x = self.mpcs[id_prec].get_assumed_state()

            self.mpcs[vehicle_id].compute_optimal_trajectories(
                self.vopt, preceding_x)

            acc = self.mpcs[vehicle_id].get_instantaneous_acceleration()

            # TEST
            p1 = self.positions[id_prec]
            p2 = self.positions[vehicle_id]
            t = self.pt.get_distance([p1[0], p1[1]], [p2[0], p2[1]])/p2[3]
            print('id = {}, v = {:.1f}, a = {:.1f}'.format(vehicle_id, v, acc))
            #print('Timegap {:.1f}'.format(t))
            # TEST

            return acc

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

    def set_reference_path(self, pt):
        """Sets a new reference ellipse path. """
        self.pt = pt

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
    pts = 400

    # Constant velocity of lead vehicle.
    v = 1.*scale

    # PID parameters.
    # 0.0001
    k_p = 0.00003
    k_i = -0.02*0
    k_d = 0.020

    if scale == 1:
        k_p = 0.5
        k_i = 0
        k_d = 3

    rate = 20

    horizon = 5
    delta_t = 0.2
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.05
    s0 = 0.
    v0 = 1.
    Q_v = 1     # Part of Q matrix for velocity tracking.
    Q_s = 0.5   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2) # State tracking.
    QN = Q
    R_acc = 0.5*0
    R = numpy.array([1]) * R_acc  # Input tracking.
    v_min = 0.
    v_max = 2.
    s_min = 0.
    s_max = 1000000
    acc_min = -0.5
    acc_max = 0.5
    truck_length = 0.5
    safety_distance = 0.3
    timegap = 0.5

    x0 = numpy.array([s0, v0])
    xmin = numpy.array([v_min, s_min])
    xmax = numpy.array([v_max, s_max])
    umin = numpy.array([acc_min])
    umax = numpy.array([acc_max])

    # Test optimal speed profile.
    opt_step = 20
    opt_v = 1.
    pos = opt_v * delta_t * numpy.arange(opt_step)
    vel = opt_v * numpy.ones(opt_step)
    vopt = speed.Speed(pos, vel)

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    # Initialize controller.
    controller = Controller(
        position_topic_type, position_topic_name,
        speed_topic_type, speed_topic_name,
        omega_topic_type, omega_topic_name, vehicle_ids, pt,
        Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
        safety_distance, timegap,
        node_name=node_name,
        v=v, k_p=k_p, k_i=k_i, k_d=k_d, rate=rate,
        xmin=xmin, xmax=xmax,
        umin=umin, umax=umax, x0=x0, QN=QN
    )

    controller.set_vopt(vopt)

    # Start controller.
    controller.run()


if __name__ == '__main__':
    main(sys.argv)
