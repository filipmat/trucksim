#!/usr/bin/env python

import rospy
import sys
import time

import path
import frenetpid
import controllerGUI
import trxmodel

from trucksim.msg import MocapState, PWM


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, vehicle_id, position_topic_name, position_topic_type,
                 pwm_topic_type, pwm_topic_name,
                 v_ref=1., k_p=0., k_i=0., k_d=0., delta_t=0.1):

        self.v_ref = v_ref              # Desired velocity of the vehicle.

        self.vehicle_id = vehicle_id    # ID of the vehicle.

        self.running = False            # If controller is running or not.

        self.dt = delta_t               # Control rate, used by GUI.

        self.zero_velocity_pwm = 1500
        self.zero_angle_pwm = 1500
        self.gear_command = 120         # UNUSED.

        self.speed_pwm = self.zero_velocity_pwm
        self.angle_pwm = self.zero_angle_pwm

        # Setup ROS node.
        rospy.init_node('controller', anonymous=True)

        # Subscribe to topic that publishes vehicle position.
        rospy.Subscriber(position_topic_name, position_topic_type, self._callback)

        # Publish vehicle pwm commands.
        self.pub_pwm = rospy.Publisher(pwm_topic_name, pwm_topic_type, queue_size=1)

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controller.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        # Keep track of the most recent vehicle pose.
        self.pose = [0., 0., 0., 0.]

        print('\nController initialized. Truck {}.\n'.format(self.vehicle_id))

    def _callback(self, data):
        """Called when the subscriber receives data. Store the vehicle pose. """
        if data.id == self.vehicle_id:
            self.pose = [data.x, data.y, data.yaw, data.v]

    def control(self):
        """Perform control actions from received data. Sends new values to truck. """

        if self.running:
            # Get control input.
            omega = self.frenet.get_omega(self.pose[0], self.pose[1], self.pose[2], self.pose[3])
            speed = self.v_ref

            self.angle_pwm = trxmodel.angular_velocity_to_steering_input(omega, speed)
            self.speed_pwm = trxmodel.linear_velocity_to_throttle_input(speed)

            # Publish control commands to the topic.
            self.publish_values()

            # Display control error.
            #print('Control error: {:5.2f}'.format(self.frenet.get_y_error()))

    def publish_values(self):
        """Publish the control values to the topic. """
        self.pub_pwm.publish(self.vehicle_id, self.speed_pwm, self.angle_pwm, self.gear_command)

    def stop(self):
        """Stops/pauses the controller. """
        self.speed_pwm = self.zero_velocity_pwm

        self.publish_values()
        time.sleep(0.05)
        self.publish_values()

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
        """Runs the controller. Used if not GUI. """
        self.start()
        rospy.spin()
        self.stop()

    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters. """
        self.frenet.set_pid(kp, ki, kd)

    def set_speed(self, v):
        """Sets the vehicle speed. """
        self.v_ref = v

    def get_adjustables(self):
        return [], []

    def set_adjustables(self):
        pass


def main(args):
    # ID of the vehicle.
    if len(args) > 1:
        vehicle_id = args[1]
    else:
        print('Need to enter a vehicle ID. ')
        sys.exit()

    position_topic_name = 'mocap_state'
    position_topic_type = MocapState

    pwm_topic_name = 'pwm_commands'
    pwm_topic_type = PWM

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0, -y_radius]

    # Constant velocity of vehicle.
    v_ref = 1

    # PID parameters for path tracking.
    k_p = 0.5
    k_i = -0.02
    k_d = 3

    delta_t = 0.1   # Update interval.

    # Initialize controller.
    controller = Controller(vehicle_id,
                            position_topic_name, position_topic_type,
                            pwm_topic_type, pwm_topic_name,
                            v_ref=v_ref, k_p=k_p, k_i=k_i, k_d=k_d, delta_t=delta_t)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    # Start controller in a GUI.
    controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
