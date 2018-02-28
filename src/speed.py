import numpy


class Speed(object):

    def __init__(self, positions = None, velocities = None):
        if positions is None or velocities is None:
            self.pos = []
            self.vel = []
        elif len(positions) == len(velocities):
            self.pos, self.vel = self.get_sorted_lists(positions, velocities)
        else:
            print('in init(): unable to set lists of different lengths. ')
            self.pos = []
            self.vel = []

    def set_new(self, positions, velocities):
        """Sets new positions and corresponding velocities. """
        try:
            if len(positions) == len(velocities):
                self.pos, self.vel = self.get_sorted_lists(positions, velocities)
            else:
                print('in set_new(): unable to set lists of different lengths. ')
                self.pos = []
                self.vel = []
        except TypeError:
            self.pos = [positions]
            self.vel = [velocities]

    def append(self, position, velocity):
        """Appends the new position and velocity values to the lists. """
        try:
            if len(position) == len(velocity):
                self.pos.extend(position)
                self.vel.extend(velocity)
            else:
                print('in append(): unable to add lists of different lengths. ')
        except TypeError:
            self.pos.append(position)
            self.vel.append(velocity)

        if len(self.pos) > 1 and self.pos[-1] < self.pos[-2]:
            self.pos, self.vel = self.get_sorted_lists(self.pos, self.vel)


    @staticmethod
    def get_sorted_lists(list_a, list_b):
        """Returns two lists that are sorted according to the elements in
        list_a. """
        if len(list_a) != len(list_b):
            return list_a, list_b

        srt = zip(*sorted(zip(list_a, list_b)))

        return list(srt[0]), list(srt[1])

    def get_speed_at(self, position):
        """Returns the velocity at the given position. If position is a list
        or a numpy array the velocities at each position are returned in a list
        or array. """
        try:
            amount = len(position)
        except TypeError:
            return self._get_speed_at(position)

        vs = [0 for i in position]
        for i in range(amount):
            vs[i] = self._get_speed_at(position[i])

        if isinstance(position, numpy.ndarray):
            return numpy.array(vs)

        return vs

    def _get_speed_at(self, position):
        """Returns the velocity at the given position."""
        if len(self.vel) == 0:
            return 0
        if len(self.vel) == 1 or position <= self.pos[0]:
            return self.vel[0]
        if position >= self.pos[-1]:
            return self.vel[-1]

        i = 1
        while i < len(self.pos) and self.pos[i] < position:
            i += 1

        try:
            k = (self.vel[i] - self.vel[i - 1])/(self.pos[i] - self.pos[i - 1])
            v = self.vel[i - 1] + k*(position - self.pos[i - 1])
        except IndexError:
            v = self.vel[-1]
        except ZeroDivisionError:
            v = self.vel[i]

        return v

    def __str__(self):
        return '(pos, vel): ' + str(zip(self.pos, self.vel))


def main():
    pos = [0., 1., 2.1, 3.4, 0.5]
    vel = [1., 2., 1.5, 1., 3.]

    vopt = Speed(pos, vel)


if __name__ == '__main__':
    main()
