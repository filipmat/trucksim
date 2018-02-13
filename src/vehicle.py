#!/usr/bin/env python

import math
import time

class Vehicle(object):
    """Class representing a point mass vehicle. """
    def __init__(self, x = [0, 0, 0], u = [0, 0]):
        self.x = x
        self.u = u

        self.last_time = time.time()
        self.last_x = self.x


    def move(self, delta_t, u = None):
        """Moves the vehicle. The vehicle moves as a point mass. If the input
        u is not specified it will use the previous input. """
        if u is not None:
            self.set_input(u)

        self.last_x = self.x[:]

        self.x[0] = self.x[0] + delta_t*self.u[0]*math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t*self.u[0]*math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t*self.u[1]) % (2*math.pi)


    def _get_velocity(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        distance = math.sqrt(
            (self.x[0] - self.last_x[0])**2 + (self.x[1] - self.last_x[1])**2)

        vel = distance/elapsed_time

        return vel


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


    def set_speed(self, v):
        self.u[0] = v


    def set_omega(self, omega):
        self.u[1] = omega


    def __str__(self):
        """Returns the current state in string format. """
        str = 'x = ['
        for x in self.x:
            str += '{:.2f}, '.format(x)

        str = str[:-2]
        str += ']'

        return str
