import numpy
import math


class Speed(object):

    def __init__(self, positions = None, velocities = None):
        if positions is None or velocities is None:
            self.pos = []
            self.vel = []
        elif len(positions) == len(velocities):
            self.pos, self.vel = self._get_sorted_lists(positions, velocities)
        else:
            print('in init(): unable to set lists of different lengths. ')
            self.pos = []
            self.vel = []

        self.repeating = False
        self.period = 1

    def set_new(self, positions, velocities):
        """Sets new speed profile with specified positions and corresponding velocities. """
        try:
            if len(positions) == len(velocities):
                self.pos, self.vel = self._get_sorted_lists(positions, velocities)
            else:
                print('in set_new(): unable to set lists of different lengths. ')
                self.pos = []
                self.vel = []
        except TypeError:
            self.pos = [positions]
            self.vel = [velocities]

        self.repeating = False

    def generate_sin(self, vmin, vmax, period, pts):
        """Generates a speed profile following a sine curve. """
        increase = float(period) / pts
        offset = float(vmax + vmin) / 2
        amplitude = float(vmax - vmin) / 2

        self.pos = [i*increase for i in range(pts)]
        self.vel = [offset + amplitude * math.sin(pos * 2 * math.pi / period) for pos in self.pos]

        self.repeating = True
        self.period = period

    @staticmethod
    def _get_sorted_lists(list_a, list_b):
        """Returns two lists that are sorted according to the elements in list_a. """
        if len(list_a) != len(list_b):
            return list_a, list_b

        srt = zip(*sorted(zip(list_a, list_b)))

        return list(srt[0]), list(srt[1])

    def get_speed_at(self, position):
        """Returns the velocity at the given position. If position is a list or a numpy array the
        velocities at each position are returned in a list or array. """
        if self.repeating:
            vs = numpy.interp(position, self.pos, self.vel, period=self.period)
        else:
            vs = numpy.interp(position, self.pos, self.vel)

        return vs

    def __str__(self):
        return '(pos, vel): ' + str(zip(self.pos, self.vel))

    def get_average(self):
        """Returns the average velocity of the speed profile. """
        average = float(sum(self.vel))/len(self.vel)

        return average
