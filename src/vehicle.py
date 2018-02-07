#!/usr/bin/env python

import math

class Vehicle(object):

    def __init__(self, x = [0, 0, 0], u = [0, 0]):
        self.x = x
        self.u = u


    def move(self, delta_t, u = None):
        if u is not None:
            self.set_input(u)

        self.x[0] = self.x[0] + delta_t*self.u[0]*math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t*self.u[0]*math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t*self.u[1]) % (2*math.pi)


    def get_state(self):
        return self.x


    def set_state(self, x):
        self.x = x


    def get_input(self):
        return self.u


    def set_input(self, u):
        self.u = u
        

    def __str__(self):
        str = 'x = ['
        for x in self.x:
            str += '{:.2f}, '.format(x)

        str = str[:-2]
        str += ']'

        return str


def main():
    pass


if __name__ == '__main__':
    main()
