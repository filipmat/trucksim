#!/usr/bin/env python

import rosbag
import yaml

import numpy

from matplotlib import pyplot

bag_name = 'measurement_vehicle_state00.bag'

bag = rosbag.Bag(bag_name, 'r')

info_dict = yaml.load(bag._get_yaml_info())

messages = info_dict['topics'][0]['messages']

x = numpy.zeros(messages)
y = numpy.zeros(messages)
yaw = numpy.zeros(messages)
yaw_rate = numpy.zeros(messages)
v = numpy.zeros(messages)
a = numpy.zeros(messages)
r = numpy.zeros(messages)
steering_command = numpy.zeros(messages)
velocity_command = numpy.zeros(messages)
gear_command = numpy.zeros(messages)
timestamp = numpy.zeros(messages)

start_time = info_dict['start']

i = 0
for topic, msg, t in bag.read_messages(topics=['TrxVehicle1']):
    x[i] = msg.x
    y[i] = msg.y
    yaw[i] = msg.yaw
    yaw_rate[i] = msg.yaw_rate
    v[i] = msg.v
    a[i] = msg.a
    r[i] = msg.r
    steering_command[i] = msg.steering_command
    velocity_command[i] = msg.velocity_command
    gear_command[i] = msg.gear_command
    timestamp[i] = t.to_sec() - start_time

    i += 1
    if i == 1:
        print(type(msg))

#pyplot.plot(steering_command, r, 'bo')
#pyplot.show()
