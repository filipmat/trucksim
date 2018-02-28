import numpy
import scipy.sparse as sparse

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

        self.nx = self.Ad.shape[0]
        self.nu = self.Bd.shape[1]

        self.pos_ref = numpy.zeros(self.h + 1)
        self.vel_ref = numpy.zeros(self.h + 1)
        self.acc_ref = numpy.zeros(self.h)
        self.preceding_pos = numpy.zeros(self.h*2)
        self.preceding_vel = numpy.zeros(self.h*2)

        self.x0 = 0
        self.zeta = 0.5
        self.Q = numpy.eye(2)
        self.R = numpy.array([1])*0.1
        self.QN = numpy.zeros((2, 2))

        self.xref = numpy.zeros(self.h*self.nx)
        self.xnref = numpy.zeros(2)
        self.uref = numpy.zeros(self.h*self.nu)
        self.xgapref = numpy.zeros(self.h*self.nx)
        self.xngapref = numpy.zeros(2)

    def compute_references(self, s0, v_opt):
        """Computes the different reference signals. """
        self.compute_pos_ref(s0, v_opt)
        self.compute_vel_ref(v_opt)
        self.compute_acc_ref()

    def compute_pos_ref(self, s0, v_opt):
        """Computes the position reference trajectory. """
        self.pos_ref[0] = s0

        for i in range(1, self.h + 1):
            self.pos_ref[i] = self.pos_ref[i - 1] + \
                              self.dt*v_opt.get_speed_at(self.pos_ref[i - 1])

    def compute_vel_ref(self, v_opt):
        """Computes the velocity reference trajectory. """
        self.vel_ref = v_opt.get_speed_at(self.pos_ref)

    def compute_acc_ref(self):
        """Computes the acceleration reference trajectory. """
        self.acc_ref = (self.vel_ref[1:] - self.vel_ref[:-1])/self.dt

    def get_assumed_trajectories(self):
        """Returns the previous states and the planned states. """
        return numpy.zeros(self.h*2)

    def get_dynamics_constraints(self):
        """Returns the constraints corresponding to the system dynamics. """
        left = sparse.kron(sparse.eye(self.h + 1), -sparse.eye(self.nx)) + \
               sparse.kron(sparse.eye(self.h + 1, k=-1), self.Ad)
        right = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.h)),
                                           sparse.eye(self.h)]),
                            self.Bd)
        matrix = sparse.hstack([left, right])

        upper = numpy.hstack([-self.x0,
                              numpy.zeros(self.h * self.nx)])

        lower = numpy.hstack([-self.x0,
                              numpy.zeros(self.h * self.nx)])

        return matrix, lower, upper

    def get_state_constraints(self, xmin, xmax):
        """Returns the constraints corresponding to the speed and position
        limits. """
        left = sparse.eye((self.h + 1) * self.nx)
        right = sparse.csc_matrix(((self.h + 1)*self.nx, self.h*self.nu))
        matrix = sparse.hstack([left, right])

        upper = numpy.tile(xmax, self.h + 1)
        lower = numpy.tile(xmin, self.h + 1)

        return matrix, upper, lower

    def get_safety_constraints(self):
        """Returns the constraints corresponding to safety distance. """
        pass

    def get_acceleration_constraints(self, umin, umax):
        """Returns the constraints corresponding to the acceleration limits. """
        left = sparse.csc_matrix((self.h*self.nu, (self.h + 1)*self.nx))
        right = sparse.eye(self.h*self.nu)
        matrix = sparse.hstack([left, right])

        upper = numpy.tile(umax, self.h)
        lower = numpy.tile(umin, self.h)

        return matrix, upper, lower

    def get_reference_cost_matrices(self):
        """Returns the cost matrices corresponding to the reference tracking."""
        p_matrix = sparse.block_diag([
            sparse.kron(sparse.eye(self.h),
                        (1 - self.zeta)*self.Q),
            (1 - self.zeta)*self.QN,
            sparse.kron(sparse.eye(self.h), self.R)
        ])

        q_vector = numpy.hstack([
            numpy.kron(numpy.eye(self.h), -self.Q).dot(self.xref),
            numpy.array([-self.QN.dot(self.xnref)]),
            numpy.kron(numpy.eye(self.h), -self.R).dot(self.uref)
        ])

        return p_matrix, q_vector

    def get_timegap_cost_matrices(self):
        """Returns the cost matrices corresponding to the timegap tracking. """
        p_matrix = sparse.block_diag([
            sparse.kron(sparse.eye(self.h), self.zeta*self.Q),
            self.zeta*self.QN,
            numpy.zeros((self.h*self.nu, self.h*self.nu))
        ])

        q_vector = numpy.hstack([
            numpy.kron(numpy.eye(self.h), -self.zeta*self.Q).dot(self.xgapref),
            numpy.array([-self.zeta*self.QN.dot(self.xngapref)]),
            numpy.zeros(self.h*self.nu)
        ])

        return p_matrix

def main():
    pos = [0., 1., 2.1, 3.4, 0.5]
    vel = [1., 2., 1.5, 1., 3.]

    vopt = speed.Speed(pos, vel)

    tr = Truck(0.5, 6)
    tr.compute_references(2, vopt)


if __name__ == '__main__':
    main()
