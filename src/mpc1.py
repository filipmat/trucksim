import cvxpy
import numpy


class MPC(object):

    def __init__(self):

        horizon = 10
        delta_t = 0.5
        x0 = numpy.array([0, 0])
        v_value = 1.
        x_min = numpy.array([0, 0])
        x_max = numpy.array([3, 2*v_value*horizon])
        u_min = numpy.array([-2])
        u_max = numpy.array([2])

        self.A = numpy.matrix([[1, 0], [delta_t, 1]])
        self.B = numpy.matrix([delta_t, 0])

        self.horizon = horizon
        self.delta_t = delta_t
        self.x0 = x0

        v_ref = v_value*numpy.ones(horizon)
        s_ref = v_value*numpy.arange(horizon)

        self.x_ref = numpy.vstack([v_ref, s_ref])
        self.u_min = u_min
        self.u_max = u_max
        self.x_min = x_min
        self.x_max = x_max





def main():
    mpc = MPC()


if __name__ == '__main__':
    main()
