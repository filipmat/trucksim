#!/usr/bin/env python

import rospy
import math
import sys
import time

from trucksim.msg import vehicleposition
from trucksim.msg import vehiclecontrol
import path
import frenetpid

class Controller():
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """
    def __init__(self, position_topic_type, position_topic_name,
        control_topic_type, control_topic_name, vehicle_id,
        node_name = 'controller', v = 1, k_p = 0, k_i = 0, k_d = 0):

        self.v = v                      # Desired velocity of the vehicle.

        self.vehicle_id = vehicle_id    # ID of the vehicle.

        self.running = False            # If controller is running or not.

        # Setup ROS node.
        rospy.init_node(node_name, anonymous = True)

        # Subscriber for vehicle positions.
        rospy.Subscriber(
            position_topic_name, position_topic_type, self._callback)

        # Publisher for controlling vehicle.
        self.pub = rospy.Publisher(control_topic_name, control_topic_type,
            queue_size = 1)

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controller.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        print('\nController initialized. Truck {}.\n'.format(self.vehicle_id))


    def _callback(self, data):
        """Called when the subscriber receives data. """
        # Retrieve data.
        vehicle_id = data.id
        x = data.x
        y = data.y
        theta = data.theta
        vel = data.v

        # Perform control if the vehicle ID is the same as in the controller.
        if vehicle_id == self.vehicle_id:
            self._control(x, y, theta, vel)


    def _control(self, x, y, theta, vel):
        """Perform control actions from received data. Sends new values to
        truck. """
        if self.running:
            # Get control input.
            omega = self.frenet.get_omega(x, y, theta, vel)

            # Publish control commands to the topic.
            self.pub.publish(self.vehicle_id, self.v, omega)

            # Display control error.
            print('Control error: {:.3f}'.format(self.frenet.get_y_error()))


    def stop(self):
        """Stops/pauses the controller. """
        t = 0.5

        self.pub.publish(self.vehicle_id, 0, 0)
        time.sleep(t)
        self.pub.publish(self.vehicle_id, 0, 0)

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


    def set_reference_path(self, radius, center = [0, 0], pts = 400):
        """Sets a new reference ellipse path. """
        self.pt.gen_circle_path(radius, pts, center)


    def run(self):
        """Runs the controller. """
        self.start()
        rospy.spin()
        self.stop()


    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters. """
        self.frenet.set_pid(kp, ki, kd)


    def set_speed(self, v):
        """Sets the vehicle speed. """
        self.v = v


def main(args):
    # ID of the vehicle.
    vehicle_id = '/' + args[1]

    # Name of ROS node.
    node_name = 'controller'

    # Topic information for subscribing to truck positions.
    position_topic_name = 'vehicle_position'
    position_topic_type = vehicleposition

    # Topic information for publishing vehicle commands.
    control_topic_name = 'vehicle_control'
    control_topic_type = vehiclecontrol

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

    # Initialize controller.
    controller = Controller(
        position_topic_type, position_topic_name, control_topic_type,
        control_topic_name, vehicle_id, node_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    # Start controller.
    controller.run()



if __name__ == '__main__':
    main(sys.argv)
