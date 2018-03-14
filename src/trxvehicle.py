

#!/usr/bin/env python
import rospy
import numpy
import sys
import rosbag

from trx_control.msg import VehicleState, MocapState, PWM
from trx_control.srv import SetMeasurement

from std_msgs.msg import Int32, String, Float32

class TrxVehicle():
    def __init__(self, vehicle_id, x = 0., y = 0., yaw = 0., v = 0., steering = 0.,
                 axel_length = 0.33):
        self.bag1 = rosbag.Bag('measurement_angle.bag', 'w')
        self.bag2 = rosbag.Bag('measurement_steering.bag', 'w')

        self.record_done = False


        self.vehicle_name = self.__class__.__name__ + str(vehicle_id)
        self.axel_length = axel_length

        self.x = x
        self.y = y
        self.yaw = yaw

        self.steering_command = 1500
        self.velocity_command = 1500
        self.gear_command = 120

        rospy.init_node(self.vehicle_name)
        self.vehicle_state_pub = rospy.Publisher(self.vehicle_name+'/VehicleState', VehicleState,
                                                 queue_size = 1)
        self.pwm_pub = rospy.Publisher('pwm_commands', PWM, queue_size = 1)
        rospy.Subscriber(self.vehicle_name + '/MocapState', MocapState, self.update_vehicle_state)

        rospy.Subscriber('teleop', PWM, self.teleop)
        rospy.Service('set_measurement', SetMeasurement, self.set_measurement)

        self.angle_x = 0
        rospy.spin()

    def teleop(self, data):
        self.steering_command = data.angle
        self.velocity_command = data.velocity
        self.gear_command = data.gear

    def set_measurement(self, req):
        self.angle_x = req.steering
        self.steering_command = 1495 - 5.016 * self.angle_x -0.2384 * self.angle_x**2
        #self.steering_command = int(req.steering)
        self.velocity_command = int(req.velocity)
        self.record_done = req.log

        if int(req.gear) == 1:
            self.gear_command = 120
        elif int(req.gear) == 2:
            self.gear_command = 60
        return True

    def update_vehicle_state(self, data):
        self.x = data.x
        self.y = data.y
        self.yaw = data.yaw

        yaw_rate = data.yaw_rate
        v = data.v

        steering_angle = numpy.arctan(yaw_rate * self.axel_length / v) * 180 / numpy.pi

        msg = VehicleState(self.vehicle_name, self.x, self.y, self.yaw, yaw_rate, v,
                           self.steering_command, self.velocity_command, steering_angle)

        self.vehicle_state_pub.publish(msg)

        if self.record_done == False:
            msg = Float32()
            msg.data = steering_angle
            self.bag1.write('angle', msg)
            msg.data = self.angle_x
            self.bag2.write('steering', msg)

        if self.record_done == True:
            self.bag1.close()
            self.bag2.close()
            self.steering_command = 1500
            self.velocity_command = 1500

        self.sender_commands()

    def sender_commands(self):
        msg = PWM()
        msg.velocity = self.velocity_command
        msg.angle = self.steering_command
        msg.gear = self.gear_command

        self.pwm_pub.publish(msg)


if __name__ == '__main__':
    vehicle = TrxVehicle(1)
