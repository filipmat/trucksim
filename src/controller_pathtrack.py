#!/usr/bin/env python

import rospy
import sys
import time

import path
import frenetpid
import controllerGUI
from trucksim.msg import vehicleposition
from trucksim.msg import vehiclespeed
from trucksim.msg import vehicleomega
import translator


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, vehicle_id, position_topic_name, position_topic_type,
                 speed_topic_type, speed_topic_name,
                 omega_topic_type, omega_topic_name,
                 v=1., k_p=0., k_i=0., k_d=0., delta_t=0.1):

        self.v = v  # Desired velocity of the vehicle.

        self.vehicle_id = vehicle_id  # ID of the vehicle.

        self.running = False  # If controller is running or not.

        self.dt = delta_t   # Used by GUI.

        self.translator = translator.Translator()

        # Setup ROS node.
        rospy.init_node('controller', anonymous=True)

        rospy.Subscriber(position_topic_name, position_topic_type, self._callback)

        # TODO: Publisher for publishing control inputs.
        self.pub_speed = rospy.Publisher(speed_topic_name, speed_topic_type, queue_size=1)
        self.pub_omega = rospy.Publisher(omega_topic_name, omega_topic_type, queue_size=1)

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controller.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        self.pose = [0., 0., 0., 0.]

        print('\nController initialized. Truck {}.\n'.format(self.vehicle_id))

    def _callback(self, data):
        """Called when the subscriber receives data. Store the vehicle pose. """
        if data.id == self.vehicle_id:
            self.pose = [data.x, data.y, data.theta, data.v]

    def control(self):
        """Perform control actions from received data. Sends new values to truck. """

        if self.running:
            # Get control input.
            omega = self.frenet.get_omega(self.pose[0], self.pose[1], self.pose[2], self.pose[3])
            speed = self.v

            # Publish control commands to the topic.
            # TODO: Send translated inputs to the vehicle.
            omega_pwm = self.translator.get_omega_value(self.pose[3], omega)
            speed_pwm = self.translator.get_speed_value(speed)

            self.pub_omega.publish(self.vehicle_id, omega)
            self.pub_speed.publish(self.vehicle_id, speed)

            # Display control error.
            print('Control error: {:5.2f}'.format(self.frenet.get_y_error()))

    def stop(self):
        """Stops/pauses the controller. """

        # TODO: Stop the vehicle.
        self.pub_speed.publish(self.vehicle_id, 0)
        self.pub_omega.publish(self.vehicle_id, 0)

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
        self.v = v

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

    position_topic_name = 'vehicle_position'
    position_topic_type = vehicleposition

    # TODO: Topic information for sending data.
    speed_topic_name = 'vehicle_speed'
    speed_topic_type = vehiclespeed
    omega_topic_name = 'vehicle_omega'
    omega_topic_type = vehicleomega

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0, -y_radius]

    # Constant velocity of vehicle.
    v = 1

    # PID parameters.
    k_p = 0.5
    k_i = -0.02
    k_d = 3

    delta_t = 0.1

    # Initialize controller.
    controller = Controller(vehicle_id,
                            position_topic_name, position_topic_type,
                            speed_topic_type, speed_topic_name,
                            omega_topic_type, omega_topic_name,
                            v=v, k_p=k_p, k_i=k_i, k_d=k_d, delta_t=delta_t)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    # Start controller.
    ctrl_gui = controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
