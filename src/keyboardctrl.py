#!/usr/bin/env python


"""
Class for controlling one vehicle with the keyboard.
"""


import rospy
import curses
import sys

from trucksim.msg import PWM


class CursesControl(object):
    """Class for controlling a vehicle with the keyboard. """
    def __init__(self, topic_type, topic_name, vehicle_id,
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

        self.gear1_command = 60
        self.gear2_command = 120

        self.init_gear_command = self.gear2_command
        self.init_gear = 2

        self.gear = self.init_gear
        self.gear_command = self.init_gear_command

        self.h = 7

        self.velocity = self.velocity_zero
        self.angle = self.angle_zero

        # ROS node for publishing values.
        rospy.init_node('keyboard_control', anonymous = True)
        self.pub = rospy.Publisher(topic_name, topic_type, queue_size = 1)

        self.stdscr = curses.initscr()

    def publish_values(self):
        """Publishes the values to the topic. """
        self.pub.publish(self.vehicle_id, self.velocity, self.angle, self.gear_command)

    def run(self):
        """Runs the curses window that captures keypresses. """
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.refresh()

        self.stdscr.addstr(0, 10, 'Arrow keys or WASD to move vehicle. ')
        self.stdscr.addstr(1, 10, 'PGUP/PGDN to change gear. ')
        self.stdscr.addstr(2, 10, 'E to stop vehicle. Q to quit program.')
        self.stdscr.addstr(3, 10, 'Sending to vehicle {}.'.format(self.vehicle_id))
        self.stdscr.addstr(self.h + 6, 20, '       ')

        self.set_velocity()
        self.set_angle()
        self.set_gear()

        key = ''
        while key != ord('q'):
            key = self.stdscr.getch()
            self.stdscr.refresh()

            if key == ord('e'):
                self.reset()
                self.publish_values()

            elif key == curses.KEY_UP or key == ord('w'):
                self.velocity = self.velocity + self.velocity_step
                self.set_velocity()
                self.publish_values()

            elif key == curses.KEY_DOWN or key == ord('s'):
                self.velocity = self.velocity - self.velocity_step
                self.set_velocity()
                self.publish_values()

            elif key == curses.KEY_LEFT or key == ord('a'):
                self.angle = self.angle - self.angle_step
                self.set_angle()
                self.publish_values()

            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.angle = self.angle + self.angle_step
                self.set_angle()
                self.publish_values()

            elif key == curses.KEY_NPAGE:
                self.gear = 1
                self.set_gear()
                self.publish_values()

            elif key == curses.KEY_PPAGE:
                self.gear = 2
                self.set_gear()
                self.publish_values()

        curses.endwin()

        self.velocity = self.velocity_zero
        self.angle = self.angle_zero
        self.gear_command = self.init_gear_command

        self.publish_values()

    def reset(self):
        """Resets to the initial values. """
        self.velocity = self.velocity_zero
        self.angle = self.angle_zero
        self.gear_command = self.init_gear_command

        self.set_velocity()
        self.set_angle()
        self.set_gear()

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
        self.stdscr.addstr(self.h, 40, '{:.2f}'.format(self.velocity))
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
            txt = 'turning left'
        else:
            txt = 'turning right '

        self.stdscr.addstr(self.h + 2, 20, txt)
        self.stdscr.addstr(self.h + 2, 40, '{:.2f}'.format(self.angle))
        self.stdscr.addstr(self.h + 6, 30, '       ')

    def set_gear(self):
        """Sets the gear of the truck. """
        if self.gear == 1:
            self.gear_command = self.gear1_command
        elif self.gear == 2:
            self.gear_command = self.gear2_command

        self.stdscr.addstr(self.h + 4, 20, 'gear')
        self.stdscr.addstr(self.h + 4, 40, '{} ({:3.0f})'.format(self.gear, self.gear_command))
        self.stdscr.addstr(self.h + 6, 30, '       ')

    @staticmethod
    def sign(x):
        if x < 0:
            return -1
        else:
            return 1


def main(screen):
    topic_type = PWM
    topic_name = 'pwm_commands'

    velocity_zero = 1500
    velocity_min = 900
    velocity_max = 2100
    velocity_step = 20

    angle_zero = 1500
    angle_min = 900
    angle_max = 2100
    angle_step = 20

    if len(sys.argv) > 1:
        vehicle_id = sys.argv[1]
    else:
        print('Need to enter a vehicle ID. ')
        sys.exit()

    curses_ctrl = CursesControl(topic_type, topic_name, vehicle_id,
                                velocity_zero, velocity_min, velocity_max, velocity_step,
                                angle_zero, angle_min, angle_max, angle_step)
    curses_ctrl.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        print('Interrupted with keyboard.')
