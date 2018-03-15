#!/usr/bin/env python

import rosbag
from std_msgs.msg import Float32

bag = rosbag.Bag('measurement_vehicle_state.bag')

i = 0
print('reading')

for topic, msg, t in bag.read_messages(topics=['v1']):
    print('{} {}'.format(t, msg))
    i += 1

    if i == 200:
        break

print('done')