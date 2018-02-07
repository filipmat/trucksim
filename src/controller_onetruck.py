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
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, node_name, position_topic_type, position_topic_name,
        control_topic_type, control_topic_name,
        v = 0, k_p = 0, k_i = 0, k_d = 0, truck_id = 2):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['k_p', 'k_i', 'k_d', 'v']

        # Velocity of the truck and PID parameters.
        self.v = v

        self.stop_angle = 0

        self.truck_id = truck_id

        # Radii and center for reference path ellipse.
        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

        self.running = False    # Controlling if controller is running or not.

        # Setup node.
        rospy.init_node(node_name, anonymous = True)

        # Subscriber for vehicle positions.
        rospy.Subscriber(
            position_topic_name, position_topic_type, self._callback)

        # Publisher for controlling vehicles.
        self.pub = rospy.Publisher(control_topic_name, control_topic_type,
            queue_size = 1)

        # Create reference path object.
        self.pt = path.Path()

        # Create frenet controller.
        self.frenet = frenetpid.FrenetPID(self.pt, k_p, k_i, k_d)

        print('\nController initialized. Truck {}.\n'.format(self.truck_id))


    def _callback(self, data):
        """Called when the subscriber receives data. """

        x = data.x
        y = data.y
        yaw = data.theta
        vel = 1

        self._control(x, y, yaw, vel)


    def _control(self, x, y, yaw, vel):
        """Perform control actions from received data. Sends new values to
        truck. """
        if self.running:

            omega = self.frenet.get_omega(x, y, yaw, vel)

            self.pub.publish(self.truck_id, 1, omega)

            self.stop_angle = 0

            print(self.frenet.get_y_error())


    def stop(self):
        """Stops/pauses the controller. """
        t = 0.05

        self.pub.publish(self.truck_id, 0, self.stop_angle)
        time.sleep(t)
        self.pub.publish(self.truck_id, 0, self.stop_angle)

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


    def set_adjustables(self, values):
        """Used by the GUI to set the adjustable values. values is a list with
        the same size as the list returned by get_adjustables(). The values
        should here be treated in the same order as specified in that list. """
        try:
            k_p = float(values[0])
            k_i = float(values[1])
            k_d = float(values[2])
            v = float(values[3])

        except:
            print('\nInvalid control parameters entered.')
            return

        self.frenet.set_pid(k_p, k_i, k_d)
        self.v = v
        self.v_pwm = self.translator.get_speed(self.v)

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        k_p, k_i, k_d = self.frenet.get_pid()

        return self.adjustables, [k_p, k_i, k_d, self.v]


    def set_reference_path(self, radius, center = [0, 0], pts = 400):
        """Sets a new reference ellipse path. """
        if isinstance(radius, list):
            if len(radius) > 1:
                self.xr = radius[0]
                self.yr = radius[1]
            else:
                self.xr = radius[0]
                self.yr = radius[0]
        else:
            self.xr = radius
            self.yr = radius

        self.xc = center[0]
        self.yc = center[1]
        self.pt.gen_circle_path([self.xr, self.yr], pts, [self.xc, self.yc])


    def run(self):
        """Runs the controller. Needs to be called if not using the GUI. """
        self.start()
        rospy.spin()
        self.stop()


def main(args):
    truck_id = '/v1'

    # Information for controller subscriber.
    node_name = 'controller'
    position_topic_name = 'vehicle_position'
    position_topic_type = vehicleposition

    control_topic_name = 'vehicle_control'
    control_topic_type = vehiclecontrol

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    # Controller tuning variables.
    v = 0.89             # Velocity used by translator model.

    k_p = 0.5
    k_i = -0.02
    k_d = 3

    # Initialize controller.
    controller = Controller(
        node_name, position_topic_type, position_topic_name,
        control_topic_type, control_topic_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d,
        truck_id = truck_id)

    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    controller.run()



if __name__ == '__main__':
    main(sys.argv)
