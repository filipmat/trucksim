#!/usr/bin/env python

import rospy
import rosbag

import sys
import os

from trucksim.msg import VehicleState, MocapState, PWM
from trucksim.srv import SetMeasurement

from geometry_msgs.msg import Twist


class TrxVehicle(object):

    def __init__(self, vehicle_id, control_frequency=20,
                 pwm_topic_name='pwm_commands', mocap_topic_name='mocap_state',
                 trx_topic_name='cmd_vel'):

        self.bag_filename_prefix = 'measurement_' + vehicle_id + '_'
        self.bag_filename_suffix = '.bag'

        logging_service_name = vehicle_id + '/set_measurement'

        self.bag_filename = self.get_filename(self.bag_filename_prefix, self.bag_filename_suffix,
                                              padding=2)

        self.recording = False

        self.vehicle_id = vehicle_id

        self.steering_command = 1500
        self.velocity_command = 1500
        self.gear_command = 120

        rospy.init_node(self.vehicle_id)

        # Currently unused publisher that publishes the state and commands.
        self.vehicle_state_pub = rospy.Publisher('vehicle_state', VehicleState, queue_size = 1)

        # Subscribe to MoCap topic.
        rospy.Subscriber(mocap_topic_name, MocapState, self.update_vehicle_state)

        # Publish data to the physical vehicle.
        self.trx_publisher = rospy.Publisher(trx_topic_name, Twist, queue_size=1)

        # Subscribe to PWM signals from controller (e.g. keyboardctrl.py).
        rospy.Subscriber(pwm_topic_name, PWM, self.teleop)

        # Service for starting or stopping recording.
        rospy.Service(logging_service_name, SetMeasurement, self.set_measurement)

        # Frequency to send control commands at.
        self.control_rate = rospy.Rate(control_frequency)

        # Loop control command sender.
        self.run()

    def start_recording(self):
        """Starts the recording. """
        self.bag_filename = self.get_filename(
            self.bag_filename_prefix, self.bag_filename_suffix, padding=2)
        self.state_bag = rosbag.Bag(self.bag_filename, 'w')
        self.recording = True
        print('Recording to {}'.format(self.bag_filename))

    def stop_recording(self):
        """Stops the recording. """
        self.recording = False
        try:
            self.state_bag.close()
            print('Recording finished in {}'.format(self.bag_filename))
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

    def teleop(self, data):
        """Controller sets pwm values. """
        if self.vehicle_id == data.id:
            self.steering_command = data.angle
            self.velocity_command = data.velocity
            self.gear_command = data.gear           # 60 or 120.

    def set_measurement(self, req):
        """Service to stop recording. """

        # Stop recording.
        if self.recording and not req.log:
            self.stop_recording()

        # Start recording.
        elif (not self.recording) and req.log:
            self.start_recording()

        return True

    def run(self):
        """Send commands to vehicle while ROS is running. """
        while not rospy.is_shutdown():
            self.send_commands()
            self.control_rate.sleep()

    def update_vehicle_state(self, data):
        """Records the vehicle state and commands. """
        msg = VehicleState(self.vehicle_id,
                           data.x, data.y, data.yaw, data.yaw_rate, data.v, data.a, data.r,
                           self.steering_command, self.velocity_command, self.gear_command)

        self.vehicle_state_pub.publish(msg)

        if self.recording:
            try:
                self.state_bag.write(self.vehicle_id, msg)
            except ValueError as e:
                print('Error: tried to write to closed bag: {}'.format(e))

    def send_commands(self):
        """Sends commands to vehicle. """
        trx_msg = Twist()
        trx_msg.linear.x = self.velocity_command
        trx_msg.angular.z = self.steering_command

        self.trx_publisher.publish(trx_msg)

        # TODO: Send gear values.


def main(args):

    if len(args) < 2:
        print('Need to enter vehicle ID. ')
        sys.exit()

    vehicle_id = args[1]
    trx_topic_name = 'cmd_vel'

    try:
        if int(args[2]) == 1:
            trx_topic_name = vehicle_id + '/' + trx_topic_name
            print('Simulated vehicle. ')
    except:
        print(args)

    pwm_topic_name = 'pwm_commands'
    mocap_topic_name = 'mocap_state'


    control_frequency = 20

    vehicle = TrxVehicle(vehicle_id, control_frequency,
                         pwm_topic_name=pwm_topic_name, mocap_topic_name=mocap_topic_name,
                         trx_topic_name=trx_topic_name)


if __name__ == '__main__':
    main(sys.argv)