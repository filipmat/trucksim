#!/usr/bin/env python

"""
Centralized MPC controller.
"""

import rospy
import rosbag
import sys
import numpy
import os

import speed_profile
import path
import frenetpid
import solver_centralized_mpc
import trxmodel

from trucksim.msg import MocapState, PWM, ControllerRun
from trucksim.srv import SetMeasurement


class CentralizedMPC(object):

    def __init__(self, position_topic_name, control_topic_name, vehicle_ids,
                 vehicle_path, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, k_p, k_i, k_d,
                 xmin=None, xmax=None, umin=None, umax=None, vopt=None,
                 recording_service_name='cmpc/set_measurement'):

        rospy.init_node('centralized_mpc', anonymous=True)

        self.dt = delta_t
        self.vehicle_ids = vehicle_ids

        self.running = False                # If controller is running or not.

        self.recording = False
        self.bag_filename_prefix = 'cmpc'
        self.bag_filename = self.get_filename(self.bag_filename_prefix, '.bag', padding=2)

        # Variables for starting phase.
        self.starting_phase_duration = 2.
        self.starting_phase = True
        self.starting_phase_start_time = 0

        # Variables for printing information in terminal.
        self.verbose = True
        self.k = 0
        print_interval = 1.     # How often to print things in terminal.
        self.print_interval_samples = int(print_interval/self.dt)
        self.control_iteration_time_sum = 0

        # Update rate.
        self.rate = rospy.Rate(1./self.dt)

        # Optimal speed profile in space. If none given, optimal speed is 1 m/s everywhere.
        if vopt is None:
            vopt = speed_profile.Speed([1], [1])
        self.vopt = vopt

        # Centralized MPC solver.
        self.mpc = solver_centralized_mpc.MPC(len(self.vehicle_ids), Ad, Bd, self.dt, horizon, zeta,
                                              Q, R, truck_length, safety_distance, timegap,
                                              xmin, xmax, umin, umax)

        self.frenets = dict()           # Path tracking.
        self.path_positions = dict()    # Longitudinal path positions.
        self.poses = dict()             # [x, y, yaw, v]
        self.speed_pwms = dict()        # Speed control signals.
        self.angle_pwms = dict()        # Wheel angle control signals.
        self.first_callbacks = dict()   # If having received first position data in initialization.

        # Initialize dictionaries.
        for vehicle_id in self.vehicle_ids:
            self.frenets[vehicle_id] = frenetpid.FrenetPID(vehicle_path, k_p, k_i, k_d)
            self.path_positions[vehicle_id] = path.PathPosition(vehicle_path)
            self.poses[vehicle_id] = [0, 0, 0, 0]
            self.speed_pwms[vehicle_id] = 1500
            self.angle_pwms[vehicle_id] = 1500
            self.first_callbacks[vehicle_id] = True

        # Publisher for controlling vehicles.
        self.pwm_publisher = rospy.Publisher(control_topic_name, PWM, queue_size=1)

        # Subscriber for vehicle positions.
        rospy.Subscriber(position_topic_name, MocapState, self._position_callback)

        # Subscriber for starting and stopping the controller.
        rospy.Subscriber('global/run', ControllerRun, self._start_stop_callback)

        # Service for starting or stopping recording.
        rospy.Service(recording_service_name, SetMeasurement, self.set_measurement)

        print('\nCentralized MPC initialized. Vehicles: {}.\n'.format(self.vehicle_ids))

    def control(self):
        """Performs one control iteration. If in the starting phase it checks if the starting phase
        is over. """
        if not self.running:
            return

        if self.starting_phase and self.k == 0:
            print('Starting phase ({:.1f} seconds) ...'.format(self.starting_phase_duration))

        # Starting phase is over after a certain duration if all vehicles were found.
        if (self.starting_phase and
                rospy.get_time() - self.starting_phase_start_time > self.starting_phase_duration):

            if True in self.first_callbacks.values():
                print('Could not obtain initial positions of all vehicles during initialization. ')
                self.stop()
            else:
                self.starting_phase = False
                self._order_follower_path_positions()
                print('Controller running ... ')

        if not self.starting_phase:
            self._control()

        self.k += 1

    def _control(self):
        start_time = rospy.get_time()   # Used for checking average iteration time.

        # String for printing information.
        info = ''
        if self.k % self.print_interval_samples == 0 and self.verbose:
            print_info = True
            avg_time = self.control_iteration_time_sum / self.print_interval_samples
            info += '\nk = {:3.0f}, average time = {:.3f} - - - - - - - - - - '.format(
                self.k, avg_time)
            self.control_iteration_time_sum = 0
        else:
            print_info = False

        x0s = self._get_x0s()  # Get initial conditions.

        self.mpc.solve_mpc(self.vopt, x0s)  # Solve MPC problem.

        accelerations = self.mpc.get_instantaneous_accelerations()  # Get accelerations.

        # For each vehicle translate acceleration into speed control input. Get steering control
        # input from Frenet controller.
        for i, vehicle_id in enumerate(self.vehicle_ids):
            # Get velocity from acceleration and velocity control input from vehicle model.
            x = self.poses[vehicle_id]

            v = self._get_vel(vehicle_id, accelerations[i])
            self.speed_pwms[vehicle_id] = self._get_throttle_input(vehicle_id, v)

            # Get angular velocity from Frenet controller and steering input from vehicle model.
            omega = self._get_omega(vehicle_id, accelerations[i])
            # TODO: check which variant of v to use.
            vo = x[3]   # Current velocity.
            # vo = v      # Target velocity according to MPC.
            # vo = trxmodel.throttle_input_to_linear_velocity(self.speed_pwms[vehicle_id])
            self.angle_pwms[vehicle_id] = trxmodel.angular_velocity_to_steering_input(omega, vo)

            # Record data.
            pos = self.path_positions[vehicle_id].get_position()

            timegap = 0
            if i > 0 and x[3] != 0:
                timegap = (self.path_positions[self.vehicle_ids[i - 1]].get_position() - pos) / x[3]

            if self.recording:
                # TODO: finish
                self._record_data()

            # Add entries to information string.
            if print_info:
                info += '\n{}: v = {:.2f} ({:.2f}), a = {:5.2f}'.format(
                    vehicle_id, self.poses[vehicle_id][3],
                    self.vopt.get_speed_at(pos),
                    accelerations[i])

                if i > 0:
                    info += ', t_gap = {:.2f}'.format(timegap)

        self._publish_vehicle_commands()


        if print_info:
            print(info)

        self.control_iteration_time_sum += rospy.get_time() - start_time

    def _get_x0s(self):
        """Returns a stacked vector with the initial condition x0 = [v0, s0] for each vehicle. """
        x0s = numpy.zeros(2*len(self.vehicle_ids))
        for i, vehicle_id in enumerate(self.vehicle_ids):
            x0s[i*2] = self.poses[vehicle_id][3]
            x0s[i*2 + 1] = self.path_positions[vehicle_id].get_position()

        return x0s

    def _get_omega(self, vehicle_id, acceleration):
        """Returns the control input omega for the specified vehicle. """
        # TODO: check which variant of speed measurement gives correct path tracking.
        pose = self.poses[vehicle_id]

        v = pose[3]
        # v = trxmodel.throttle_input_to_linear_velocity(self.speed_pwms[vehicle_id])
        # v = pose[3] + self.dt * acceleration

        omega = self.frenets[vehicle_id].get_omega(pose[0], pose[1], pose[2], v)

        return omega

    def _get_vel(self, vehicle_id, acceleration):
        """Returns the new target velocity from the acceleration and current control signal. """
        vel = self.poses[vehicle_id][3] + acceleration * self.dt

        return vel

    def _get_throttle_input(self, vehicle_id, new_vel):
        """Returns the new control input for the vehicle. """
        pwm_diff = trxmodel.linear_velocity_to_throttle_input(new_vel) - \
                   trxmodel.linear_velocity_to_throttle_input(self.poses[vehicle_id][3])
        speed_pwm = self.speed_pwms[vehicle_id] + pwm_diff

        return speed_pwm

    def _order_follower_path_positions(self):
        """Fixes the path longitudinal positions so that each follower vehicle is positioned behind
        the preceding vehicle. """
        for i, vehicle_id in enumerate(self.vehicle_ids):

            if i > 0:
                preceding_position = self.path_positions[self.vehicle_ids[i - 1]].get_position()

                if (self.path_positions[vehicle_id].get_position() - 0.01 > preceding_position or
                        self.path_positions[vehicle_id].get_position() + 0.01 < preceding_position):

                    self.path_positions[vehicle_id].set_position_behind(preceding_position)

    def _publish_vehicle_commands(self):
        """Publishes the current PWM values for speed and wheel angle for each vehicle. """
        for vehicle_id in self.vehicle_ids:
            self.pwm_publisher.publish(vehicle_id, self.speed_pwms[vehicle_id],
                                       self.angle_pwms[vehicle_id], 120)
        # TODO: change gear value.

    def _position_callback(self, data):
        """Callback for subscriber subscribing to vehicle positions. """
        if data.id not in self.vehicle_ids:
            return

        self.poses[data.id] = [data.x, data.y, data.yaw, data.v]

        # If in the starting phase, only update the path position the first time.
        if self.starting_phase:
            if self.first_callbacks[data.id]:
                self.path_positions[data.id].update_position([data.x, data.y])
                self.first_callbacks[data.id] = False

        else:
            self.path_positions[data.id].update_position([data.x, data.y])

    def _start_stop_callback(self, data):
        """Callback for the subscriber on topic used for starting or stopping the controller. """
        if data.run:
            self.start()
        else:
            self.stop()

        return 1

    def start(self):
        """Starts the controller. Starts the starting phase and sets velocity to zero. """
        if not self.running:
            self.running = True

            self.k = 0

            self.starting_phase_start_time = rospy.get_time()
            self.starting_phase = True

            for vehicle_id in self.vehicle_ids:
                self.first_callbacks[vehicle_id] = True
                self.speed_pwms[vehicle_id] = 1500

            self._publish_vehicle_commands()

            print('Controller started. ')
        else:
            print('Controller is already running. ')

    def stop(self):
        """Stops the controller. """
        if self.running:
            self.running = False
            print('Controller stopped. \n')

        for i in range(2):  # Do several times to ensure that the vehicles stop.
            for vehicle_id in self.speed_pwms:
                self.speed_pwms[vehicle_id] = 1500
                self._publish_vehicle_commands()
                self.rate.sleep()

    def run(self):
        """Runs the controller. Used when global GUI. """
        while not rospy.is_shutdown():
            self.control()
            self.rate.sleep()

        self.stop()

    def _record_data(self):
        if self.recording:
            try:
                #self.bag.write('cmpc', tuple(self.vehicles_information))
                pass
            except ValueError as e:
                print('Error when writing to bag: {}'.format(e))

    def set_measurement(self, req):
        """Service callback to start and stop recording. """

        # Stop recording.
        if self.recording and not req.log:
            self.stop_recording()

        # Start recording.
        elif (not self.recording) and req.log:
            self.start_recording()

        return True

    def start_recording(self):
        """Starts the recording. """
        self.recording = True

        self.bag_filename = self.get_filename(self.bag_filename_prefix, '.bag', padding=2)
        self.bag = rosbag.Bag(self.bag_filename, 'w')

        print('Recording to {}.'.format(self.bag_filename))

    def stop_recording(self):
        """Stops the recording. """
        self.recording = False

        try:
            self.bag.close()
            print('Recording finished in {}.'.format(self.bag_filename))
        except NameError as e:
            print('Error when stopping recording: {}'.format(e))

    @staticmethod
    def get_filename(prefix, suffix, padding=0):
        """Sets a filename on the form filename_prefixZ.bag where Z is the first free number.
        Pads with zeros, e.g. first free number 43 and padding=5 will give 00043. """
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

        i = 0
        while os.path.exists(os.path.join(__location__, '{}{}{}'.format(
                prefix, str(i).zfill(padding), suffix))):
            i += 1

        filename = os.path.join(__location__, '{}{}{}'.format(
            prefix, str(i).zfill(padding), suffix))

        return filename


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

    # Name for starting and stopping recording.
    recording_service_name = 'cmpc/set_measurement'

    # PID parameters for path tracking.
    k_p = 0.5
    k_i = 0.001
    k_d = 3

    horizon = 20
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.5
    Q_v = 1  # Part of Q matrix for velocity tracking.
    Q_s = 1  # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)  # State tracking.
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    velocity_min = 0.
    velocity_max = 2.
    position_min = -100000.
    position_max = 1000000.
    acceleration_min = -0.5
    acceleration_max = 0.5
    truck_length = 0.3
    safety_distance = 0.2
    timegap = 1.

    xmin = numpy.array([velocity_min, position_min])
    xmax = numpy.array([velocity_max, position_max])
    umin = numpy.array([acceleration_min])
    umax = numpy.array([acceleration_max])

    # Reference speed profile.
    opt_v_pts = 400  # How many points.
    opt_v_max = 1.2
    opt_v_min = 0.8
    opt_v_period_length = 60  # Period in meters.
    vopt = speed_profile.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True
    # vopt = speed_profile.Speed([1], [1])

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
                         umax=umax, vopt=vopt, recording_service_name=recording_service_name)

    mpc.run()


if __name__ == '__main__':
    main(sys.argv)