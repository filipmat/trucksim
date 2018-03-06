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
        str += ' {:.2f}'.format(v)
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
                 node_name='controller', v=1., k_p=0., k_i=0., k_d=0.,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None,
                 saved_h=2):

        self.verbose = True

        self.v = v  # Desired velocity of the vehicle.
        self.vehicle_ids = ['/' + v_id for v_id in vehicle_ids]  # IDs.
        self.running = False  # If controller is running or not.
        self.rate = 1/delta_t

        self.dt = delta_t
        self.headstart_samples = int(1./self.dt)
        self.h = horizon

        self.vopt = speed.Speed()

        self.truck_length = truck_length

        self.k = 0
        self.last_control_time = time.time()

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
                                                      x0=x0,
                                                      vehicle_id=vehicle_id,
                                                      saved_h=saved_h)
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

        self._control_startup()
        self._control_normal_operation()


    def _control_startup(self):

        v = self.vopt.get_average()

        for j in range(len(self.vehicle_ids)):

            if self.running:

                for i in range(self.headstart_samples):

                    for vehicle_id in self.vehicle_ids:
                        omega = self._get_omega(vehicle_id)
                        self.pub_omega.publish(vehicle_id, omega)

                    if i < self.headstart_samples - 3:
                        acc = v/((self.headstart_samples - 3)*self.dt)
                    else:
                        acc = 0

                    self.pub_speed.publish(self.vehicle_ids[j], acc)

                    time.sleep(self.dt)
                    print('{:2.0f}/{}: Starting {}'.format(
                        i, self.headstart_samples - 1, self.vehicle_ids[j]))

    def _control_normal_operation(self):
        k_skip = round(1./self.dt)

        while not rospy.is_shutdown():
            if self.running:
                time_start = time.time()
                if self.verbose and self.k % k_skip == 0:
                    tm = time.time() - self.last_control_time
                    self.last_control_time = time.time()
                    print('\nk = {}, avg time = {:.3f}, dt = {:.2f}, diff = {:.3f}'.format(
                        self.k, tm/k_skip, self.dt, tm/k_skip - self.dt))

                for vehicle_id in self.vehicle_ids:
                    omega = self._get_omega(vehicle_id)
                    self.pub_omega.publish(vehicle_id, omega)

                    acc = self._get_acc(vehicle_id)

                    self.pub_speed.publish(vehicle_id, acc)

                self.k += 1

                t_elapsed = time.time() - time_start
                if (t_elapsed < self.dt):
                    time.sleep(self.dt - t_elapsed)



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

        # Leader vehicle: don't add preceding vehicle information.
        if self.vehicle_ids.index(vehicle_id) == 0:
            self.mpcs[vehicle_id].compute_optimal_trajectories(self.vopt)

        else:
            id_prec = self.vehicle_ids[self.vehicle_ids.index(vehicle_id) - 1]
            preceding_x = self.mpcs[id_prec].get_assumed_state()

            self.mpcs[vehicle_id].compute_optimal_trajectories(
                self.vopt, preceding_x)

        acc = self.mpcs[vehicle_id].get_instantaneous_acceleration()

        # Print stuff
        if self.verbose and self.k % round(1./self.dt) == 0:
                opt_v = self.vopt.get_speed_at(path_pos)

                s = ''
                s += 'id = {}, s = {:6.2f}'.format(vehicle_id, path_pos)
                s += ', v = {:.2f} ({:.2f}), a = {:5.2f}'.format(
                    v, opt_v, acc)

                if self.vehicle_ids.index(vehicle_id) > 0:
                    p1 = self.path_positions[id_prec].get_position()
                    p2 = self.path_positions[vehicle_id].get_position()
                    d = p1 - p2 - self.truck_length
                    try:
                        t = d / v
                    except ZeroDivisionError:
                        t = 0
                    s += ', timegap = {:.2f}'.format(t)

                    if self.mpcs[vehicle_id].status != 'OK':
                        s += '. ' + self.mpcs[vehicle_id].status


                print(s)

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

    horizon = 5
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.5
    s0 = 0.
    v0 = 1.
    Q_v = 1     # Part of Q matrix for velocity tracking.
    Q_s = 0.5   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2) # State tracking.
    QN = Q
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    v_min = 0.
    v_max = 2.
    s_min = 0.
    s_max = 1000000
    acc_min = -0.5
    acc_max = 0.5
    truck_length = 0.2
    safety_distance = 0.1
    timegap = 1.
    saved_h = 2

    x0 = numpy.array([s0, v0])
    xmin = numpy.array([v_min, s_min])
    xmax = numpy.array([v_max, s_max])
    umin = numpy.array([acc_min])
    umax = numpy.array([acc_max])

    # Reference speed profile.
    opt_v_pts = 1000
    opt_v_max = 1.3
    opt_v_min = 0.7
    opt_v_period_length = 100
    vopt = speed.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

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
        v=v, k_p=k_p, k_i=k_i, k_d=k_d,
        xmin=xmin, xmax=xmax,
        umin=umin, umax=umax, x0=x0, saved_h=saved_h
    )

    controller.set_vopt(vopt)

    # Start controller.
    controller.run()


if __name__ == '__main__':
    main(sys.argv)
