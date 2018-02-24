import math


class FrenetPID(object):
    def __init__(self, path, k_p=0, k_i=0, k_d=0):
        # PID parameters.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.ey = 0  # Current error.
        self.sumy = 0  # Accumulated error.

        # Reference path.
        self.pt = path

    def get_omega(self, x, y, yaw, vel):
        """Calculate the control input omega. """

        index, closest = self.pt.get_closest([x, y])  # Closest point on path.

        self.ey = self.pt.get_ey([x, y])  # y error (distance from path)

        self.sumy = self.sumy + self.ey  # Accumulated error.

        gamma = self.pt.get_gamma(index)
        gamma_p = self.pt.get_gammap(index)
        gamma_pp = self.pt.get_gammapp(index)

        cos_t = math.cos(yaw - gamma)  # cos(theta)
        sin_t = math.sin(yaw - gamma)  # sin(theta)

        # y prime (derivative w.r.t. path).
        yp = math.tan(yaw - gamma) * (1 - gamma_p * self.ey) * self._sign(
            vel * cos_t / (1 - gamma_p * self.ey))

        # PID controller.
        u = - self.k_p * self.ey - self.k_d * yp - self.k_i * self.sumy

        # Feedback linearization.
        omega = vel * cos_t / (1 - gamma_p * self.ey) * (
                u * cos_t ** 2 / (1 - gamma_p * self.ey) +
                gamma_p * (1 + sin_t ** 2) +
                gamma_pp * self.ey * cos_t * sin_t / (1 - gamma_p * self.ey))

        return omega

    @staticmethod
    def _sign(x):
        """Returns the sign of x. """
        if x > 0:
            return 1
        else:
            return -1

    def set_pid(self, kp=None, ki=None, kd=None):
        """Sets the PID parameters. """
        if kp is not None:
            self.k_p = kp
        if ki is not None:
            self.k_i = ki
        if kd is not None:
            self.k_d = kd

        self.reset_sum()

    def get_pid(self):
        """Returns the PID parameters. """
        return self.k_p, self.k_i, self.k_d

    def reset_sum(self):
        """Resets the sum for I part in PID controller. """
        self.sumy = 0

    def update_path(self, path):
        """Updates the reference path. """
        self.pt = path

    def get_y_error(self):
        """Returns the latest y error. """
        return self.ey
