#!/usr/bin/env python

import rospy
import curses
import sys


class CursesControl(object):
    """Class for controlling a vehicle with the keyboard. """
    def __init__(self, node_name, topic_type, topic_name, vehicle_id,
                 velocity_zero, velocity_min, velocity_max, velocity_step,
                 angle_zero, angle_min, angle_max, angle_step):

        self.vehicle_id = vehicle_id

        self.velocity_zero = velocity_zero
        self.velocity_min = velocity_min
        self.velocity_max = velocity_max
        self.velocity_step = velocity_step

        self.angle_zero = angle_zero
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_step = angle_step

        self.h = 7

        self.velocity = self.velocity_zero
        self.angle = self.angle_zero

        # ROS node for publishing values.
        rospy.init_node(node_name, anonymous = True)
        # self.pub = rospy.Publisher(truck_topic_name, truck_topic_type, queue_size = 1)

        self.stdscr = curses.initscr()

    def publish_values(self, velocity, angle):
        """Publishes the values to the topic. """
        # self.pub.publish(self.vehicle_id, velocity, angle)
        pass

    def run(self):
        """Runs the curses window that captures keypresses. """
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.refresh()

        self.stdscr.addstr(0, 10, 'Arrow keys or WASD to move vehicle. ')
        self.stdscr.addstr(1, 10, 'E to stop vehicle. Q to quit program.')
        self.stdscr.addstr(2, 10, 'Sending to vehicle {}'.format(self.vehicle_id))
        self.stdscr.addstr(self.h + 6, 20, '       ')

        self.set_velocity()
        self.set_angle()

        key = ''
        while key != ord('q'):
            key = self.stdscr.getch()
            self.stdscr.refresh()

            if key == ord('e'):
                self.reset()
                # self.publish_values(self.velocity, self.angle)

            if key == curses.KEY_UP or key == ord('w'):
                self.velocity = self.velocity + self.velocity_step
                self.set_velocity()
                # self.publish_values(self.velocity, self.angle)

            elif key == curses.KEY_DOWN or key == ord('s'):
                self.velocity = self.velocity - self.velocity_step
                self.set_velocity()
                # self.publish_values(self.velocity, self.angle)

            if key == curses.KEY_LEFT or key == ord('a'):
                self.angle = self.angle - self.angle_step
                self.set_angle()
                # self.publish_values(self.velocity, self.angle)

            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.angle = self.angle + self.angle_step
                self.set_angle()
                # self.publish_values(self.velocity, self.angle)

        curses.endwin()
        #self.publish_values(self.zero_velocity, self.zero_angle)

    def reset(self):
        """Resets to the initial values. """
        self.velocity = self.velocity_zero
        self.angle = self.angle_zero

        self.set_velocity()
        self.set_angle()

        self.stdscr.addstr(self.h + 6, 30, 'stopped')

    def set_velocity(self):
        """Sets the velocity of the truck. """
        if self.velocity < self.velocity_min:
            self.velocity = self.velocity_min
        if self.velocity > self.velocity_max:
            self.velocity = self.velocity_max

        if self.velocity == self.velocity_zero:
            txt = 'standing still  '
        elif self.sign(self.velocity - self.velocity_zero) == self.sign(self.velocity_step):
            txt = 'driving forward '
        else:
            txt = 'driving backward'

        self.stdscr.addstr(self.h, 20, txt)
        self.stdscr.addstr(self.h, 40, '%.2f' % self.velocity)
        self.stdscr.addstr(self.h + 6, 30, '       ')

    def set_angle(self):
        """Sets the angle of the truck wheels. """
        if self.angle < self.angle_min:
            self.angle = self.angle_min
        if self.angle > self.angle_max:
            self.angle = self.angle_max

        if self.angle == self.angle_zero:
            txt = 'straight     '
        elif self.sign(self.angle - self.angle_zero) == self.sign(self.angle_step):
            txt = 'turning right'
        else:
            txt = 'turning left '

        self.stdscr.addstr(self.h + 2, 20, txt)
        self.stdscr.addstr(self.h + 2, 40, '%.2f' % self.angle)
        self.stdscr.addstr(self.h + 6, 30, '       ')

    @staticmethod
    def sign(x):
        if x < 0:
            return -1
        else:
            return 1


def main(screen):
    node_name = 'keyboard_control'
    topic_type = 'hej'
    topic_name = 'hej'

    velocity_zero = 1500
    velocity_min = 1200
    velocity_max = 1800
    velocity_step = -50

    angle_zero = 1500
    angle_min = 1100
    angle_max = 1900
    angle_step = -100

    if len(sys.argv) > 1:
        vehicle_id = sys.argv[1]
    else:
        print('Need to enter a vehicle ID. ')
        sys.exit()

    curses_ctrl = CursesControl(node_name, topic_type, topic_name, vehicle_id,
                                velocity_zero, velocity_min, velocity_max, velocity_step,
                                angle_zero, angle_min, angle_max, angle_step)
    curses_ctrl.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        print('Interrupted with keyboard.')
