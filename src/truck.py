import numpy

import speed

"""
On update: keep some old positions in order to send them to the following 
vehicle for timegap tracking. At least one is needed for safety constraint (not 
if safety constraint is just distance, then also constraint 2 becomes obsolete).  

Position: either position along path, or position measured relative the 
position of the leader vehicle at the current time instant. 


"""


class MPC(object):

    def __init__(self):
        pass


class Truck(object):

    def __init__(self, delta_t, horizon):
        self.dt = delta_t
        self.h = horizon

        self.Ad = numpy.matrix([[1, 0], [delta_t, 1]])
        self.Bd = numpy.matrix([[delta_t], [0]])

        self.pos_ref = numpy.zeros(self.h + 1)
        self.vel_ref = numpy.zeros(self.h + 1)
        self.acc_ref = numpy.zeros(self.h)
        self.preceding_pos = numpy.zeros(self.h*2)
        self.preceding_vel = numpy.zeros(self.h*2)

    def compute_references(self, s0, v_opt):
        """Computes the different reference signals. """
        self.compute_pos_ref(s0, v_opt)
        self.compute_vel_ref(v_opt)
        self.compute_acc_ref()

    def compute_pos_ref(self, s0, v_opt):
        """Computes the position reference trajectory. """
        self.pos_ref[0] = s0

        for i in range(1, self.h + 1):
            self.pos_ref[i] = self.pos_ref[i -1] + \
                              self.dt*v_opt.get_speed_at(self.pos_ref[i - 1])

    def compute_vel_ref(self, v_opt):
        """Computes the velocity reference trajectory. """
        self.vel_ref = v_opt.get_speed_at(self.pos_ref)

    def compute_acc_ref(self):
        """Computes the acceleration reference trajectory. """
        self.acc_ref = (self.vel_ref[1:] - self.vel_ref[:-1])/self.dt

    def get_assumed_trajectories(self):
        return numpy.zeros(self.h*2)


def main():
    pos = [0., 1., 2.1, 3.4, 0.5]
    vel = [1., 2., 1.5, 1., 3.]

    vopt = speed.Speed(pos, vel)

    tr = Truck(0.5, 6)
    tr.compute_references(2, vopt)
    print(tr.pos_ref)


if __name__ == '__main__':
    main()
