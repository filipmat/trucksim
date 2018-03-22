#!/usr/bin/env python

import rospy
import sys
import time
import numpy

from trucksim.msg import MocapState, PWM
import path
import frenetpid
import truckmpc
import speed_profile
import trxmodel
import controllerGUI


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)


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

        if self.position > self.zero_passes*self.path_length + pos + self.path_length/8:
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
                 control_topic_type, control_topic_name, vehicle_ids, vehicle_path,
                 Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, saved_h=2,
                 k_p=0., k_i=0., k_d=0.):

        self.verbose = True
        self.do_startup = False

        self.vehicle_ids = vehicle_ids
        self.running = False  # If controller is running or not.

        self.gear2_pwm = 120

        self.dt = delta_t
        self.headstart_samples = int(1./self.dt)

        self.vopt = speed_profile.Speed()

        self.truck_length = truck_length

        self.print_interval = round(1./self.dt)

        self.k = 0
        self.last_print_time = time.time()

        # Setup ROS node.
        rospy.init_node('platoon_controller', anonymous=True)

        # Publisher for controlling vehicle.
        self.pwm_publisher = rospy.Publisher(control_topic_name, control_topic_type, queue_size=1)

        # Subscriber for vehicle positions.
        rospy.Subscriber(position_topic_name, position_topic_type, self._callback)

        # Reference path.
        self.pt = vehicle_path

        self.frenets = dict()           # Frenet path tracking controllers for each vehicle.
        self.positions = dict()         # Stores vehicle positions [x, y, yaw, v].
        self.mpcs = dict()              # MPC controllers.
        self.path_positions = dict()    # For keeping track of longitudinal path positions.
        self.speed_pwms = dict()        # Stores speed PWM for each vehicle.
        for i, vehicle_id in enumerate(self.vehicle_ids):
            self.frenets[vehicle_id] = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

            self.positions[vehicle_id] = [0, 0, 0, 0]

            # The first vehicle entered is set to be the leader.
            if i == 0:
                leader = True
            else:
                leader = False

            self.mpcs[vehicle_id] = truckmpc.TruckMPC(Ad, Bd, delta_t, horizon, zeta, Q, R,
                                                      truck_length, safety_distance, timegap,
                                                      xmin=xmin, xmax=xmax, umin=umin, umax=umax,
                                                      x0=x0, vehicle_id=vehicle_id, saved_h=saved_h,
                                                      is_leader=leader)

            self.path_positions[vehicle_id] = PathPosition(self.pt)
            self.speed_pwms[vehicle_id] = 1500

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
        yaw = data.yaw
        vel = data.v

        self.positions[vehicle_id] = [x, y, yaw, vel]
        self.path_positions[vehicle_id].update_position([x, y])

    def control(self):
        """Performs one iteration of the control. It will either be startup or normal operation
        with MPC. """
        if self.running:

            if self.k < len(self.vehicle_ids)*self.headstart_samples and self.do_startup:
                self._startup()
            else:
                self._run_mpc()

    def _startup(self):
        """Runs one iteration of the startup. Accelerates one vehicle at a time depending on the
        value of k. """

        end_num = 3     # For last few samples of startup the vehicle will not accelerate.
        startup_acc = self.vopt.get_average()/((self.headstart_samples - end_num)*self.dt)

        current_vehicle_index = self.k / self.headstart_samples     # Current vehicle starting.

        for i, vehicle_id in enumerate(self.vehicle_ids):
            angle_pwm = self._get_angle_pwm(vehicle_id)     # Steering input for path tracking.

            # Accelerate the current vehicle that is started. Others have acceleration zero.
            if i == current_vehicle_index:
                if self.k % self.headstart_samples == 0:
                    print('Starting vehicle {}.'.format(vehicle_id))

                if self.k % self.headstart_samples < self.headstart_samples - 3:
                    acc = startup_acc
                else:
                    acc = 0     # End of startup.
            else:
                acc = 0         # Vehicle already started or not yet started.

            # Calculate the new speed control input from the current input and the acceleration.
            new_vel = trxmodel.throttle_input_to_linear_velocity(self.speed_pwms[vehicle_id]) + \
                acc * self.dt
            self.speed_pwms[vehicle_id] = trxmodel.linear_velocity_to_throttle_input(new_vel)

            # Publish speed and steering inputs.
            self.pwm_publisher.publish(vehicle_id, self.speed_pwms[vehicle_id], angle_pwm,
                                       self.gear2_pwm)

            # Update the MPC state information.
            vel = self.positions[vehicle_id][3]
            path_pos = self.path_positions[vehicle_id].get_position()
            self.mpcs[vehicle_id].set_new_x0(numpy.array([vel, path_pos]))

        self.k += 1

    def _run_mpc(self):
        """Runs one iteration of the MPC controller. """

        if self.verbose and self.k % self.print_interval == 0:
            elapsed_time = time.time() - self.last_print_time
            average_time = elapsed_time / self.print_interval
            self.last_print_time = time.time()
            print('\nk = {}, avg time = {:.3f}, dt = {:.2f}, diff = {:.3f}'.format(
                self.k, average_time, self.dt, average_time - self.dt))

        for vehicle_id in self.vehicle_ids:
            # Get control input for the steering from the Frenet path tracking controller.
            angle_pwm = self._get_angle_pwm(vehicle_id)

            # Get control input for the velocity from the MPC controller.
            speed_pwm = self._get_speed_pwm(vehicle_id)

            # Publish speed and steering inputs.
            self.pwm_publisher.publish(vehicle_id, speed_pwm, angle_pwm, self.gear2_pwm)

        self.k += 1

    def _get_angle_pwm(self, vehicle_id):
        """Returns the steering control input for the vehicle from the path tracking and vehicle
        model. """
        pose = self.positions[vehicle_id]
        omega = self.frenets[vehicle_id].get_omega(pose[0], pose[1], pose[2], pose[3])
        velocity = self.positions[vehicle_id][3]

        angle_pwm = trxmodel.angular_velocity_to_steering_input(omega, velocity)

        return angle_pwm

    def _get_speed_pwm(self, vehicle_id):
        """Returns the speed control input from the MPC controller. The MPC controller gives
        acceleration which is then translated to a new velocity and a control input. """
        acceleration = self._get_acceleration(vehicle_id)
        new_velocity = trxmodel.throttle_input_to_linear_velocity(self.speed_pwms[vehicle_id]) + \
            acceleration * self.dt
        self.speed_pwms[vehicle_id] = trxmodel.linear_velocity_to_throttle_input(new_velocity)

        return self.speed_pwms[vehicle_id]


    def _get_acceleration(self, vehicle_id):
        """Returns the desired acceleration of the vehicle from the MPC controller. If the vehicle
        is not the leader vehicle it will take into account the preceding vehicle's trajectory. """
        velocity = self.positions[vehicle_id][3]
        path_pos = self.path_positions[vehicle_id].get_position()

        self.mpcs[vehicle_id].set_new_x0(numpy.array([velocity, path_pos]))

        id_prec = ''    # ID of preceding vehicle.

        # Leader vehicle: don't add preceding vehicle information.
        if self.vehicle_ids.index(vehicle_id) == 0:
            self.mpcs[vehicle_id].compute_optimal_trajectories(self.vopt)
        else:
            id_prec = self.vehicle_ids[self.vehicle_ids.index(vehicle_id) - 1]
            preceding_x = self.mpcs[id_prec].get_assumed_state()

            self.mpcs[vehicle_id].compute_optimal_trajectories(self.vopt, preceding_x)

        acc = self.mpcs[vehicle_id].get_instantaneous_acceleration()

        # Print stuff
        if self.verbose and self.k % self.print_interval == 0:
            opt_v = self.vopt.get_speed_at(path_pos)

            s = ''
            s += 'id = {}, s = {:6.2f}'.format(vehicle_id, path_pos)
            s += ', v = {:.2f} ({:.2f}), a = {:6.3f}'.format(velocity, opt_v, acc)

            if self.vehicle_ids.index(vehicle_id) > 0:
                pos1 = self.path_positions[id_prec].get_position()
                pos2 = self.path_positions[vehicle_id].get_position()
                distance = pos1 - pos2 - self.truck_length
                try:
                    timegap = distance / velocity
                except ZeroDivisionError:
                    timegap = 0
                s += ', timegap = {:.2f}'.format(timegap)

                if self.mpcs[vehicle_id].status != 'OK':
                    s += '. ' + self.mpcs[vehicle_id].status

            print(s)

        return acc

    def stop(self):
        """Stops/pauses the controller. """
        for vehicle_id in self.vehicle_ids:
            self.pwm_publisher.publish(vehicle_id, 1500, 1500, 120)

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

        while not rospy.is_shutdown():
            self.control()
            time.sleep(self.dt)

        self.stop()

    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters. """
        for vehicle_id in self.vehicle_ids:
            self.frenets[vehicle_id].set_pid(kp, ki, kd)


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_ids = args[1:]

    # Topic information for subscribing to truck positions.
    position_topic_name = 'mocap_state'
    position_topic_type = MocapState

    # Topic information for publishing vehicle commands.
    control_topic_name = 'pwm_commands'
    control_topic_type = PWM

    scale = 1

    # Data for controller reference path.
    x_radius = 1.7*scale
    y_radius = 1.2*scale
    center = [0, -y_radius]
    pts = 400

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
    zeta = 0.75
    s0 = 0.
    v0 = 1.
    Q_v = 1     # Part of Q matrix for velocity tracking.
    Q_s = 0.5   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)     # State tracking.
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
    opt_v_max = 1.2
    opt_v_min = 0.8
    opt_v_period_length = 40
    vopt = speed_profile.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    # Initialize controller.
    controller = Controller(
        position_topic_type, position_topic_name, control_topic_type, control_topic_name,
        vehicle_ids, pt, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance,
        timegap, xmin=xmin, xmax=xmax, umin=umin, umax=umax, x0=x0, saved_h=saved_h,
        k_p=k_p, k_i=k_i, k_d=k_d
    )

    controller.set_vopt(vopt)

    # Start controller.
    controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
