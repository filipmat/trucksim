#!/usr/bin/env python

import rospy
import numpy
import sys
import rosbag
import time
import math

from trucksim.msg import VehicleState, MocapState, PWM
from trucksim.srv import SetMeasurement

from std_msgs.msg import Int32, String, Float32


class TrxVehicle(object):
    def __init__(self, vehicle_id, x = 0., y = 0., yaw = 0., v = 0., steering = 0.,
                 axel_length = 0.33):
        self.bag = rosbag.Bag('measurement_vehicle_state.bag', 'w')

        self.record_done = False

        self.vehicle_id = vehicle_id

        self.axel_length = axel_length

        # TODO: Change to 1500.
        self.zero_steering = 0
        self.zero_velocity = 0

        self.steering_command = self.zero_steering
        self.velocity_command = self.zero_velocity
        self.gear_command = 120

        rospy.init_node(self.vehicle_id)
        self.vehicle_state_pub = rospy.Publisher('vehicle_state', VehicleState, queue_size = 1)
        self.pwm_pub = rospy.Publisher('pwm_commands', PWM, queue_size = 1)
        rospy.Subscriber('mocap_state', MocapState, self.update_vehicle_state)

        rospy.Subscriber('teleop', PWM, self.teleop)
        rospy.Service('set_measurement', SetMeasurement, self.set_measurement)

        self.angle_x = 0
        rospy.spin()

    def teleop(self, data):
        self.steering_command = data.angle
        self.velocity_command = data.velocity
        self.gear_command = data.gear           # 60 or 120.

    def set_measurement(self, req):
        self.record_done = req.log

        if self.record_done:
            print('Record done. ')

        return True

    def update_vehicle_state(self, data):

        msg = VehicleState(self.vehicle_id,
                           data.x, data.y, data.yaw, data.yaw_rate, data.v, data.a, data.r,
                           self.steering_command, self.velocity_command, self.gear_command)

        self.vehicle_state_pub.publish(msg)

        if not self.record_done:
            self.bag.write(self.vehicle_id, msg)
        else:
            self.bag.close()
            self.steering_command = self.zero_steering
            self.velocity_command = self.zero_velocity

        self.sender_commands()

    def sender_commands(self):
        msg = PWM()
        msg.velocity = self.velocity_command
        msg.angle = self.steering_command
        msg.gear = self.gear_command

        self.pwm_pub.publish(msg)


def main(args):
    if len(args) < 2:
        print('Need to enter vehicle ID. ')
        sys.exit()

    vehicle = TrxVehicle(args[1])


if __name__ == '__main__':
    main(sys.argv)