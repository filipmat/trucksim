#!/usr/bin/env python

import math
import time

class Vehicle(object):
    """Class representing a unicycle vehicle. """
    def __init__(self, x = [0, 0, 0], u = [0, 0]):
        self.x = x  # Vehicle state.
        self.u = u  # Vehicle input.

        # Variables used for velocity calculation.
        self.delta_t = 0        # Stores the update interval.
        self.last_x = self.x[:] # Stores the last state.
        self.v = 0              # Vehicle velocity. 


    def move(self, delta_t, u = None):
        """Moves the vehicle. The vehicle moves as a unicycle. If the input
        u is not specified it will use the previous input. """
        if u is not None:
            self.set_input(u)

        # Save information for velocity calculation.
        self.last_x = self.x[:]
        self.delta_t = delta_t

        # Move the vehicle according to the dynamics.
        self.x[0] = self.x[0] + delta_t*self.u[0]*math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t*self.u[0]*math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t*self.u[1]) % (2*math.pi)


    def get_velocity(self):
        """Returns the velocity of the vehicle. The velocity is calculated
        using the distance traveled from the last position and the elapsed time
        from the last movement. The elapsed time is the update interval. """
        distance = math.sqrt(
            (self.x[0] - self.last_x[0])**2 + (self.x[1] - self.last_x[1])**2)

        # Calculate the velocity. If the latest update interval equals to zero
        # the previous velocity is used.
        try:
            self.v = distance/self.delta_t
        except ZeroDivisionError:
            pass

        return self.v


    def get_state(self):
        """Returns the current state. """
        return self.x


    def set_state(self, x):
        """Sets the state. """
        self.x = x


    def get_input(self):
        """Returns the current input. """
        return self.u


    def set_input(self, u):
        """Sets the input. """
        self.u = u


    def __str__(self):
        """Returns the current state in string format. """
        str = 'x = ['
        for x in self.x:
            str += '{:.2f}, '.format(x)

        str = str[:-2]
        str += ']'

        return str
