#!/usr/bin/env python

import rospy
import sys
import time

import path
import frenetpid
import controllerGUI


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, vehicle_id, node_name='controller', v=1., k_p=0., k_i=0., k_d=0.,
                 delta_t=0.1):

        self.v = v  # Desired velocity of the vehicle.

        self.vehicle_id = vehicle_id  # ID of the vehicle.

        self.running = False  # If controller is running or not.

        self.dt = delta_t

        # Setup ROS node.
        rospy.init_node(node_name, anonymous=True)

        # TODO: Subscriber for receiving positions, calls _callback.

        # TODO: Publisher for publishing control inputs.

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controller.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        self.pose = [0., 0., 0., 0.]

        print('\nController initialized. Truck {}.\n'.format(self.vehicle_id))

    def _callback(self, data):
        """Called when the subscriber receives data. """
        # Retrieve data.

        # TODO: Get correct information from topic.

        vehicle_id = data.id
        x = data.x
        y = data.y
        theta = data.theta
        vel = data.v

        self.pose = [x, y, theta, vel]

    def control(self):
        """Perform control actions from received data. Sends new values to truck. """

        if self.running:
            # Get control input.
            omega = self.frenet.get_omega(self.pose[0], self.pose[1], self.pose[2], self.pose[3])

            # Publish control commands to the topic.
            # TODO: Translate and send control inputs to vehicle.

            # Display control error.
            print('Control error: {:.3f}'.format(self.frenet.get_y_error()))

    def stop(self):
        """Stops/pauses the controller. """
        t = 0.5

        # TODO: Stop the vehicle.

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
    vehicle_id = '/' + args[1]

    # Name of ROS node.
    node_name = 'controller'

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    # Constant velocity of vehicle.
    v = 1

    # PID parameters.
    k_p = 0.5
    k_i = -0.02
    k_d = 3

    delta_t = 0.1

    # Initialize controller.
    controller = Controller(vehicle_id, node_name,
        v=v, k_p=k_p, k_i=k_i, k_d=k_d, delta_t=delta_t)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    # Start controller.
    ctrl_gui = controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
