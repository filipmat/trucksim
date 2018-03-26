#!/usr/bin/env python

"""
Class for controlling a single vehicle. Path tracking with Frenet and speed control with MPC.
If the vehicle is a follower it uses trajectories of the preceding vehicle.
"""

import rospy

import numpy
import sys

import speed_profile
import path
import distributed_mpc_solver
import frenetpid
import controllerGUI
import trxmodel

from trucksim.msg import MocapState, PWM, AssumedState

# TODO: maybe have different update frequencies for path tracking and velocity control.


class Controller(object):

    def __init__(self, position_topic_name, control_topic_name, vehicle_id, preceding_id,
                 is_leader, vehicle_path, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, vopt=None,
                 k_p=0., k_i=0., k_d=0.):

        self.vehicle_id = vehicle_id    # ID of the vehicle.
        self.dt = delta_t
        self.h = horizon

        # Setup ROS node.
        rospy.init_node(self.vehicle_id + '_mpc', anonymous=True)

        self.is_leader = is_leader      # True if the vehicle is the leader.

        self.pose = [0, 0, 0, 0]            # Store most recent vehicle pose.
        self.timestamp = rospy.get_time()   # Store the timestamp for the saved pose.

        # Store preceding vehicle trajectories. Store predicted horizon and old states for timegap.
        assumed_state_length = horizon + int(2./delta_t)
        self.preceding_timestamps = numpy.zeros(assumed_state_length)
        self.preceding_velocities = numpy.zeros(assumed_state_length)
        self.preceding_positions = numpy.zeros(assumed_state_length)

        self.pt = vehicle_path      # Reference path for path tracking.
        self.path_position = path.PathPosition(self.pt)  # Keep track of longitudinal path position.

        self.running = False    # Controls whether the controller is running or not.
        self.starting_phase = True
        self.starting_phase_duration = 2.
        self.first_callback = True
        self.starting_phase_start_time = 0

        # PWM values for the speed, wheel angle, and gears.
        self.speed_pwm = 1500
        self.angle_pwm = 1500
        self.gear_pwm = 120

        # Optimal speed profile in space. If none given, optimal speed is 1 m/s everywhere.
        if vopt is None:
            vopt = speed_profile.Speed([0], [1])
        self.vopt = vopt

        # Update interval.
        self.dt = delta_t
        self.rate = rospy.Rate(1. / self.dt)

        # Old and predicted velocities and positions.
        saved_states = int(timegap*1.5/delta_t) + horizon   # One horizon plus enough old states.
        self.velocities = numpy.zeros(saved_states)
        self.positions = numpy.zeros(saved_states)
        self.timestamps = numpy.zeros(saved_states)

        # Publisher for controlling vehicle.
        self.pwm_publisher = rospy.Publisher(control_topic_name, PWM, queue_size=1)

        # Subscriber for vehicle positions.
        rospy.Subscriber(position_topic_name, MocapState, self._position_callback)

        # Publisher for publishing assumed state.
        self.assumed_publisher = rospy.Publisher(self.vehicle_id + '/assumed_state', AssumedState,
                                                 queue_size=1)

        # Subscriber for receiving assumed state of preceding vehicle.
        rospy.Subscriber(preceding_id + '/assumed_state', AssumedState,
                         self._assumed_state_callback)

        # MPC controller for speed control.
        self.mpc = distributed_mpc_solver.MPC(Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                                              safety_distance, timegap, xmin=xmin, xmax=xmax,
                                              umin=umin, umax=umax, x0=x0, is_leader=is_leader)

        # Frenet controller for path tracking.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        print('Vehicle controller initialized. ID = {}.'.format(self.vehicle_id))
        if self.is_leader:
            print('Leader vehicle. ')
        else:
            print('Follower vehicle, following {}.'.format(preceding_id))

    def _assumed_state_callback(self, data):
        """Called when receiving the assumed state from the preceding vehicle. Stores the
        trajectories of the preceding vehicle. """
        self.preceding_timestamps = data.timestamps
        self.preceding_velocities = data.velocity
        self.preceding_positions = data.position

        # If in starting phase and has received first own position information, update this
        # longitudinal path position so that it is positioned after the preceding vehicle.
        if self.running and self.starting_phase and not self.first_callback and not self.is_leader:

            preceding_position = self.preceding_positions[-(self.h + 1)]

            if self.path_position.get_position() - 0.01 > preceding_position:
                self.path_position.set_position_behind(preceding_position)
                self.positions[:] = self.path_position.get_position()

    def _position_callback(self, data):
        """Called when receiving position data from MoCap or simulated vehicle. Stores the pose. """
        if data.id == self.vehicle_id:

            self.pose = [data.x, data.y, data.yaw, data.v]
            self.timestamp = data.timestamp

            if self.starting_phase:
                if self.first_callback:
                    self.path_position.update_position([data.x, data.y])
                    self.velocities[:] = data.v
                    self.positions[:] = self.path_position.get_position()
                    self.timestamps[:] = data.timestamp
                    self.first_callback = False

            else:
                self.path_position.update_position([data.x, data.y])

            try:
                self._publish_assumed_state()
            except:
                pass

    def _publish_assumed_state(self):
        """Publishes the assumed state consisting of old and predicted states. """
        msg = AssumedState()
        msg.id = self.vehicle_id
        msg.timestamps = self.timestamps
        msg.velocity = self.velocities
        msg.position = self.positions

        self.assumed_publisher.publish(msg)

    def control(self):
        """Controls the vehicle. """

        if not self.running:
            return

        if self.starting_phase:
            print('{:.1f}/{:.1f} Starting phase... '.format(
                rospy.get_time() - self.starting_phase_start_time, self.starting_phase_duration))

        if (self.starting_phase and
                rospy.get_time() - self.starting_phase_start_time > self.starting_phase_duration):
            self.starting_phase = False
            print('Controller running... ')

        if not self.starting_phase:

            self._update_current_state()

            self._solve_mpc()

            self._update_assumed_state()

            self.angle_pwm = self._get_steering_input()
            self.speed_pwm = self._get_throttle_input()

            self._publish_control_commands()

    def _update_current_state(self):
        """Shifts the old states and adds the current state. """
        self.velocities[0:-(self.h + 1)] = self.velocities[1:-self.h]
        self.positions[0:-(self.h + 1)] = self.positions[1:-self.h]
        self.timestamps[0:-(self.h + 1)] = self.timestamps[1:-self.h]

        self.velocities[-(self.h + 1)] = self.pose[3]
        self.positions[-(self.h + 1)] = self.path_position.get_position()
        self.timestamps[-(self.h + 1)] = self.timestamp

    def _solve_mpc(self):
        """Solves the MPC problem given the current state, and state of preceding vehicle if not
        the leader. """
        if self.is_leader:
            self.mpc.solve_mpc(self.vopt,
                               [self.velocities[-(self.h + 1)], self.positions[-(self.h + 1)]])
        else:
            self.mpc.solve_mpc(self.vopt,
                               [self.velocities[-(self.h + 1)], self.positions[-(self.h + 1)]],
                               self.timestamps[-(self.h + 1)],
                               self.preceding_timestamps, self.preceding_velocities,
                               self.preceding_positions)

    def _get_steering_input(self):
        """Returns the steering command. Calculated from Frenet path tracker and vehicle model. """
        omega = self.frenet.get_omega(self.pose[0], self.pose[1], self.pose[2], self.pose[3])
        steering_command = trxmodel.angular_velocity_to_steering_input(omega, self.pose[3])

        return steering_command

    def _get_throttle_input(self):
        """Returns the throttle command. Calculated from MPC and vehicle model. """
        acceleration = self.mpc.get_instantaneous_acceleration()
        new_velocity = trxmodel.throttle_input_to_linear_velocity(self.speed_pwm) + \
            acceleration * self.dt
        speed_pwm = trxmodel.linear_velocity_to_throttle_input(new_velocity)

        return speed_pwm

    def _update_assumed_state(self):
        """Updates the assumed state. Adds the predicted position and velocity trajectories to the
        assumed trajectory lists. """
        vel, pos = self.mpc.get_predicted_states()

        self.velocities[-(self.h + 1):] = vel
        self.positions[-(self.h + 1):] = pos
        self.timestamps[-(self.h + 1):] = self.timestamps[-(self.h + 1)] + \
                                          self.dt*numpy.arange(self.h + 1)

    def _publish_control_commands(self):
        """Publishes the current desired control inputs to the vehicle. """
        self.pwm_publisher.publish(self.vehicle_id, self.speed_pwm, self.angle_pwm, self.gear_pwm)

    def stop(self):
        """Stops/pauses the controller. """
        self.speed_pwm = 1500
        self._publish_control_commands()

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
            self.starting_phase_start_time = rospy.get_time()
            self.starting_phase = True
            self.first_callback = True
            self.speed_pwm = 1500
            print('Controller started.')
        else:
            print('Controller is already running. ')

    def run(self):
        """Runs the controller. Used when no GUI. """
        self.start()

        while not rospy.is_shutdown():
            self.control()
            self.rate.sleep()

        self.stop()


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_id = args[1]        # First argument is the ID of the vehicle.
    if len(args) > 2:
        preceding_id = args[2]  # Second argument is, if entered, the ID of the preceding vehicle.
        is_leader = False
    else:
        preceding_id = 'None'   # If no second argument, the vehicle is the leader.
        is_leader = True

    # Topic name for subscribing to truck positions.
    position_topic_name = 'mocap_state'

    # Topic name for publishing vehicle commands.
    control_topic_name = 'pwm_commands'

    # PID parameters for path tracking.
    k_p = 0.5
    k_i = 0
    k_d = 3

    # MPC information.
    horizon = 5
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.75
    s0 = 0.
    v0 = 0.
    Q_v = 1     # Part of Q matrix for velocity tracking.
    Q_s = 0.5   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0, 0, Q_s]).reshape(2, 2)     # State tracking.
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
    opt_v_pts = 1000            # How many points.
    opt_v_max = 1.2
    opt_v_min = 0.8
    opt_v_period_length = 40    # Period in meters.
    vopt = speed_profile.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

    # Controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0, -y_radius]
    pts = 400

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    # Initialize controller.
    controller = Controller(
        position_topic_name, control_topic_name, vehicle_id, preceding_id, is_leader,
        pt, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance, timegap,
        vopt=vopt, xmin=xmin, xmax=xmax, umin=umin, umax=umax, x0=x0,
        k_p=k_p, k_i=k_i, k_d=k_d
    )

    # Start controller.
    # controller.run()
    controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
