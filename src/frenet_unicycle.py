import frenetpid
import unicycle


class FrenetUnicycle(unicycle.Unicycle):

    def __init__(self, pt, x=None, u=None, kp=0, ki=0, kd=0, ID=None):

        super(FrenetUnicycle, self).__init__(x, u, ID)

        self.frenet = frenetpid.FrenetPID(pt, kp, ki, kd)
        self.path_pos = PathPosition(pt, [x[0], x[1]])

    def update(self, delta_t):
        """Gets a new control input and moves the vehicle. """
        self.u[1] = self._get_omega()
        self._move(delta_t)
        self.path_pos.update_position([self.x[0], self.x[1]])

    def _get_omega(self):
        """Returns the omega control signal calculated by frenet controller. """
        return self.frenet.get_omega(self.x[0], self.x[1], self.x[2], self.x[3])

    def get_path_pos(self):
        """Returns the position on the path. """
        return self.path_pos.get_position()

    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters of the frenet controller. """
        self.frenet.set_pid(kp, ki, kd)

    def get_error(self):
        """Returns the distance to the path. """
        return self.frenet.get_y_error()


class PathPosition(object):
    """Class for keeping track of absolute vehicle position on the path.
    The position increases with each lap, i.e. does not reset to zero. """
    def __init__(self, pt, xy):
        self.pt = pt
        self.position = self.pt.get_position_on_path(xy)
        self.path_length = self.pt.get_path_length()
        self.zero_passes = 0
        # Allow backwards travel distance less than a fraction of path length.
        self.backwards_fraction = 1./8

    def update_position(self, xy):
        """Updates the position on the path. """
        pos = self.pt.get_position_on_path(xy)

        if (self.position >
                self.zero_passes*self.path_length + pos + self.path_length/8):
            self.zero_passes += 1

        self.position = self.zero_passes*self.path_length + pos

    def get_position(self):
        """Returns the position on the path. """
        return self.position

    @staticmethod
    def order_positions(path_positions):
        for i in range(len(path_positions)):

            if i > 0:
                while (path_positions[i].position >
                       path_positions[i - 1].position):
                    path_positions[i].position -= path_positions[i].path_length

    def __str__(self):
        return '{:.2f}'.format(self.position)







