#!/usr/bin/env python

import rospy

from trucksim.msg import vehiclecontrol
from trucksim.msg import vehicleposition

import vehicle



class VehicleNode(vehicle.Vehicle):

    def __init__(self, x = [0, 0, 0], u = [0, 0], frequency = 20,
        name = None):

        # Node and topic names and types.
        DEFAULT_NAME = 'vehicle'
        CONTROL_TOPIC_NAME = 'vehicle_control'
        POSITION_TOPIC_NAME = 'vehicle_position'
        CONTROL_TOPIC_TYPE = vehiclecontrol
        POSITION_TOPIC_TYPE = vehicleposition

        # Initialize superclass.
        super(VehicleNode, self).__init__(x, u)

        # Subscriber for receiving control inputs.
        rospy.Subscriber(CONTROL_TOPIC_NAME, CONTROL_TOPIC_TYPE, self._callback)

        # Publisher for publishing vehicle position.
        self.pub = rospy.Publisher(
            POSITION_TOPIC_NAME, POSITION_TOPIC_TYPE, queue_size = 10)

        # Initialize ROS node.
        if name is not None:
            rospy.init_node(name, anonymous = False)
        else:
            rospy.init_node(DEFAULT_NAME, anonymous = True)

        # ROS update rate.
        self.r = rospy.Rate(frequency)

        # Update interval.
        self.delta_t = 1./frequency

        print('Vehicle node initialized, id: {}'.format(rospy.get_name()))


    def _callback(self, data):
        """Method called when subscriber receives data. Update input if the
        published ID is the same as the ID of this node. """
        if data.truck_id == rospy.get_name():
            self.set_input([data.speed, data.omega])


    def run(self):
        """Run the simulation. Moves the vehicle and publishes the position. """
        while not rospy.is_shutdown():
            self.move(self.delta_t)

            self.pub.publish(
                rospy.get_name(), self.x[0], self.x[1], self.x[2])

            self.r.sleep()


    def __str__(self):
        return rospy.get_name() + ': ' + super(VehicleNode, self).__str__()


def main():
    pass


if __name__ == '__main__':
    main()
