#!/usr/bin/env python

import math


class Unicycle(object):
    """Class representing a unicycle vehicle. The states are x, y, orientation
    and velocity. The inputs are linear acceleration and angular velocity. """

    def __init__(self, x=None, u=None, ID='vehicle'):

        if x is None:
            x = [0, 0, 0, 0]
        self.x = x              # Vehicle state.

        if u is None:
            u = [0, 0]
        self.u = u              # Vehicle input.

        self.ID = ID

    def update(self, delta_t):
        """Updates the vehicle position. """
        self._move(delta_t)

    def _move(self, delta_t):
        """Moves the vehicle according to the system dynamics. """
        self.x[0] = self.x[0] + delta_t * self.x[3] * math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t * self.x[3] * math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t * self.u[1]) % (2 * math.pi)
        self.x[3] = self.x[3] + delta_t*self.u[0]

    def get_velocity(self):
        """Returns the velocity of the vehicle. """
        return self.x[3]

    def get_x(self):
        """Returns the current state. """
        return self.x

    def set_x(self, x):
        """Sets the state. """
        self.x = x

    def get_u(self):
        """Returns the current input. """
        return self.u

    def set_u(self, u):
        """Sets the input. """
        self.u = u

    def set_omega(self, omega):
        self.u[1] = omega

    def set_acceleration(self, acc):
        self.u[0] = acc

    def __str__(self):
        """Returns the current state in string format. """
        s = 'ID = {}: x = ['.format(self.ID)
        for x in self.x:
            s += '{:.2f}, '.format(x)

        s = s[:-2]
        s += ']'

        return s
