#!/usr/bin/env python

import rospy
import curses
import sys


class CursesControl():
    """Class for controlling a truck with the keyboard. """
    def __init__(self, node_name, truck_topic_type, truck_topic_name, velocity_step=50,
                 angle_step=50, truck_id=2):
        self.vel_step = velocity_step
        self.ang_step = angle_step
        self.truck_id = truck_id

        self.init_velocity = 1500
        self.init_angle = 1500
        self.init_gearval = 60
        self.init_gear = 1

        self.angle_max = 1900
        self.angle_min = 1100

        self.velocity_max = 1800
        self.velocity_min = 1200

        self.h = 7

        self.velocity = self.init_velocity
        self.angle = self.init_angle

        rospy.init_node(node_name, anonymous = True)
        # self.pub = rospy.Publisher(truck_topic_name, truck_topic_type,
        #     queue_size = 1)

        self.stdscr = curses.initscr()

    def run(self):
        """Runs the curses window that captures keypresses. """
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.refresh()

        self.stdscr.addstr(0, 10, 'Arrow keys or WASD to move truck. T to switch truck.')
        self.stdscr.addstr(1, 10, 'E to stop truck. Q to stop truck and quit program.')
        self.stdscr.addstr(2, 10, 'Ctrl-C will quit but not stop truck.')
        self.stdscr.addstr(4, 10, 'Sending to truck {}'.format(self.truck_id))
        self.stdscr.addstr(self.h + 6, 20, '       ')

        self.set_velocity()
        self.set_angle()

        key = ''
        while key != ord('q'):
            key = self.stdscr.getch()
            self.stdscr.refresh()

            if key == ord('e'):
                self.reset()

            if key == ord('t'):
                self.switch_truck()

            if key == curses.KEY_UP or key == ord('w'):
                self.velocity = self.velocity - self.vel_step
                self.set_velocity()

            elif key == curses.KEY_DOWN or key == ord('s'):
                self.velocity = self.velocity + self.vel_step
                self.set_velocity()

            if key == curses.KEY_LEFT or key == ord('a'):
                self.angle = self.angle + self.ang_step
                self.set_angle()

            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.angle = self.angle - self.ang_step
                self.set_angle()

        curses.endwin()
        #self.pub.publish(self.truck_id, 1500, self.angle)

    def reset(self):
        #self.pub.publish(self.truck_id, 1500, 1500)

        self.velocity = self.init_velocity
        self.angle = self.init_angle

        self.set_velocity()
        self.set_angle()

        self.stdscr.addstr(11, 30, 'stopped')

    def switch_truck(self):
        """Switches truck_id from 1 to 2 or from 2 to 1. """
        self.reset()
        self.truck_id = 3 - self.truck_id
        self.stdscr.addstr(4, 10, 'Sending to truck {}'.format(self.truck_id))
        self.stdscr.addstr(self.h + 6, 30, '       ')

    def set_velocity(self):
        """Sets the velocity of the truck. """
        if self.velocity < self.velocity_min:
            self.velocity = self.velocity_min
        if self.velocity > self.velocity_max:
            self.velocity = self.velocity_max

        if self.velocity == self.init_velocity:
            txt = 'standing still  '
        elif self.velocity < self.init_velocity:
            txt = 'driving forward '
        else:
            txt = 'driving backward'

        self.stdscr.addstr(self.h, 20, txt)
        self.stdscr.addstr(self.h, 40, '%.2f' % self.velocity)
        self.stdscr.addstr(self.h + 6, 30, '       ')

        #self.pub.publish(self.truck_id, self.velocity, self.angle)

    def set_angle(self):
        """Sets the angle of the truck wheels. """
        if self.angle < self.angle_min:
            self.angle = self.angle_min
        if self.angle > self.angle_max:
            self.angle = self.angle_max

        if self.angle == self.init_angle:
            txt = 'straight     '
        elif self.angle < self.init_angle:
            txt = 'turning right'
        else:
            txt = 'turning left '

        self.stdscr.addstr(self.h + 2, 20, txt)
        self.stdscr.addstr(self.h + 2, 40, '%.2f' % self.angle)
        self.stdscr.addstr(self.h + 6, 30, '       ')

        #self.pub.publish(self.truck_id, self.velocity, self.angle)


def main(screen):
    node_name = 'keyboard_control'
    truck_topic_type = 'hej'
    truck_topic_name = 'hej'

    velocity_step = 50  # Size of velocity pwm step.
    angle_step = 100    # Size of angle pwm step.

    # Truck 2 is selected unless argument 1 is entered as argument.
    truck_id = 2
    try:
        if int(sys.argv[1]) == 1:
            truck_id = 1
    except:
        pass

    curses_ctrl = CursesControl(node_name, truck_topic_type, truck_topic_name,
        velocity_step=velocity_step, angle_step=angle_step, truck_id=truck_id)
    curses_ctrl.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        print('Interrupted with keyboard.')
