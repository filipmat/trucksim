import numpy
import scipy.sparse as sparse
import cvxpy

import speed

"""
Position: either position along path, or position measured relative the 
position of the leader vehicle at the current time instant. 
"""


class TruckMPC(object):

    def __init__(self, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, QN=None,
                 t_id=None):

        self.Ad = Ad
        self.Bd = Bd
        self.dt = delta_t
        self.h = horizon
        self.zeta = zeta
        self.Q = Q
        self.R = R
        self.truck_length = truck_length
        self.safety_distance = safety_distance
        self.timegap = timegap

        self.nx = self.Ad.shape[0]
        self.nu = self.Bd.shape[1]

        self.inf = 1000000

        self.is_leader = False

        if xmin is None:
            self.xmin = -self.inf*numpy.ones(self.nx)  # No min state limit.
        else:
            self.xmin = xmin

        if xmax is None:
            self.xmax = self.inf*numpy.ones(self.nx)  # No max state limit.
        else:
            self.xmax = xmax

        if umin is None:
            self.umin = -self.inf*numpy.ones(self.nu)  # No min input limit.
        else:
            self.umin = umin

        if umax is None:
            self.umax = self.inf*numpy.ones(self.nu)  # No max input limit.
        else:
            self.umax = umax

        if x0 is None:
            self.x0 = numpy.zeros(self.nx)  # Origin default initial condition.
        else:
            self.x0 = x0

        if QN is None:
            self.QN = numpy.zeros((self.nx, self.nx))
        else:
            self.QN = QN

        # Computed from optimal speed profile.
        self.pos_ref = numpy.zeros(self.h + 1)
        self.vel_ref = numpy.zeros(self.h + 1)
        self.acc_ref = numpy.zeros(self.h)

        # Put together from reference trajectories above.
        self.xref = numpy.zeros((self.h + 1)*self.nx)
        self.uref = numpy.zeros(self.h*self.nu)

        # Assumed state contains past states and optimal state from MPC.
        self.assumed_x = numpy.zeros(self.h * 2 * self.nx)

        # State of preceding vehicle.
        self.preceding_x = numpy.zeros(self.h * 2 * self.nx)

        self.z = cvxpy.Variable((self.h + 1)*self.nx + self.h*self.nu)


        self.testx = cvxpy.Variable((self.h + 1)*self.nx)
        self.testu = cvxpy.Variable(self.h*self.nu)

    def set_leader(self, is_leader):
        self.is_leader = is_leader

    def set_new_x0(self, x0):
        """Sets the current initial value. The positions of previous saved
        states are shifted. """
        self.x0 = x0
        self.assumed_x[:self.h * self.nx] = self.assumed_x[
                                            self.nx:(self.h + 1) * self.nx]

    def compute_optimal_trajectories(self, vopt, preceding_x=None):
        """Computes the optimal trajectories using MPC and updates the vehicle
        assumed state. """
        self.update_mpc(vopt, preceding_x)
        self.solve_mpc()
        self.update_assumed_state()

    def update_mpc(self, vopt, preceding_x):
        """Updates the MPC problem with the new initial position, optimal
        speed trajectory and preceding vehicle trajectory. """
        if preceding_x is not None:
            self.preceding_x = preceding_x
        self.compute_references(self.x0[1], vopt)

    def solve_mpc(self):
        """Solves the MPC problem. """
        cost = self.get_mpc_cost()
        objective = cvxpy.Minimize(cost)
        constraints = self.get_mpc_constraints()
        prob = cvxpy.Problem(objective, constraints)
        try:
            prob.solve(solver='CVXOPT')
            print(prob.status)
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(prob.status))

    def update_assumed_state(self):
        """Updates the assumed state. The length is two horizons. The first part
        are the previous states, the second part starts with the current state
        and then contains the optimal state from the MPC solution. """
        try:
            self.assumed_x[self.h*self.nx:] = \
                self.z.value[:self.h*self.nx].flatten()
        except TypeError:
            self.assumed_x[self.h*self.nx:] = self.get_backup_predicted_state()

    def get_backup_predicted_state(self):
        """Returns a predicted state trajectory over one horizon. Used if there
        is no optimal trajectory available from the MPC solver. """
        state = numpy.zeros(self.h*self.nx)
        state[0::2] = self.x0[0]
        state[1::2] = self.x0[1] + self.x0[0]*self.dt*numpy.arange(self.h)

        return state

    def get_mpc_cost(self):
        """Returns the cost for the MPC problem. The cost is the combined cost
        for tracking reference, timegap and input trajectories. If the vehicle
        is the leader the timegap tracking cost is excluded. """
        p_ref, q_ref = self.get_reference_cost_matrices()
        p_ref = sparse.csr_matrix(p_ref)
        for i in range(self.nx):    # No cost for initial value.
            p_ref[i, i] = 0
            q_ref[i] = 0

        print(p_ref.toarray())
        print(q_ref)

        if self.is_leader:
            cost = 0.5 * cvxpy.quad_form(self.z, p_ref) + q_ref * self.z
        else:
            p_gap, q_gap = self.get_timegap_cost_matrices()
            cost = 0.5 * cvxpy.quad_form(self.z, p_ref + p_gap) + \
                   (q_ref + q_gap) * self.z

        return cost

    def get_mpc_constraints(self):
        """Returns the constraints for the MPC problem. """
        a_dyn, upper_dyn, lower_dyn = self.get_dynamics_constraints()
        a_st, upper_st, lower_st = self.get_state_constraints()
        a_acc, upper_acc, lower_acc = self.get_acceleration_constraints()

        constraints = [a_dyn * self.z == upper_dyn,
                       a_st * self.z <= upper_st,
                       a_st * self.z >= lower_st,
                       a_acc * self.z <= upper_acc,
                       a_acc * self.z >= lower_acc]

        # ,
        # a_st * self.z <= upper_st,
        # a_st * self.z >= lower_st,
        # a_acc * self.z <= upper_acc,
        # a_acc * self.z >= lower_acc

        #a_dyn * self.z >= lower_dyn,

        if not self.is_leader: # Add safety constraint if not leader.
            a_saf, upper_saf, lower_saf = self.get_safety_constraints()

            constraints = constraints + [a_saf * self.z <= upper_saf,
                                         a_saf * self.z >= lower_saf]

        return constraints

    def compute_references(self, s0, v_opt):
        """Computes the different reference signals. """
        self.compute_pos_ref(s0, v_opt)
        self.compute_vel_ref(v_opt)
        self.compute_acc_ref()

        self.xref = self.interleave_vectors(self.vel_ref, self.pos_ref)
        self.uref = self.acc_ref[:]

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

        return matrix, upper, lower

    def get_state_constraints(self):
        """Returns the constraints corresponding to the speed and position
        limits. """
        left = sparse.eye((self.h + 1) * self.nx)
        right = sparse.csc_matrix(((self.h + 1)*self.nx, self.h*self.nu))
        matrix = sparse.hstack([left, right])

        upper = numpy.tile(self.xmax, self.h + 1)
        lower = numpy.tile(self.xmin, self.h + 1)

        return matrix, upper, lower

    def get_acceleration_constraints(self):
        """Returns the constraints corresponding to the acceleration limits. """
        left = sparse.csc_matrix((self.h*self.nu, (self.h + 1)*self.nx))
        right = sparse.eye(self.h*self.nu)
        matrix = sparse.hstack([left, right])

        upper = numpy.tile(self.umax, self.h)
        lower = numpy.tile(self.umin, self.h)

        return matrix, upper, lower

    def get_safety_constraints(self):
        """Returns the constraints corresponding to safety distance.
        pos is the assumed positions of the preceding vehicle. Length h + 1. """
        pos = self.preceding_x[(self.h - 2)*self.nx:(2*self.h - 1)*self.nx][
            1::2]

        a = numpy.array([0, 0, 0, 1]).reshape(2, 2)
        left = sparse.kron(sparse.eye(self.h + 1), a)
        right = sparse.csc_matrix(((self.h + 1)*self.nx, self.h*self.nu))
        matrix = sparse.hstack([left, right])

        upper = self.interleave_vectors(numpy.ones(self.h + 1)*self.inf, pos)
        lower = numpy.zeros(2*(self.h + 1))

        return matrix, upper, lower

    def get_reference_cost_matrices(self):
        """Returns the cost matrices corresponding to the reference tracking."""
        p_matrix = sparse.block_diag([
            sparse.kron(sparse.eye(self.h),
                        (1 - self.zeta)*self.Q),
            (1 - self.zeta)*self.QN,
            sparse.kron(sparse.eye(self.h), self.R)
        ])

        xr = self.xref[:-self.nx]

        q_vector = numpy.hstack([
            numpy.kron(numpy.eye(self.h), -self.Q).dot(xr),
            -self.QN.dot(self.xref[-self.nx:]),
            numpy.kron(numpy.eye(self.h), -self.R).dot(self.uref)
        ])

        return p_matrix, q_vector

    def get_timegap_cost_matrices(self):
        """Returns the cost matrices corresponding to the timegap tracking. """
        shift = int(round(self.timegap/self.dt))
        xgapref = self.preceding_x[
                  (self.h - shift)*self.nx:(2*self.h - shift + 1)*self.nx]

        p_matrix = sparse.block_diag([
            sparse.kron(sparse.eye(self.h), self.zeta*self.Q),
            self.zeta*self.QN,
            numpy.zeros((self.h*self.nu, self.h*self.nu))
        ])

        q_vector = numpy.hstack([
            numpy.kron(numpy.eye(self.h),
                       -self.zeta*self.Q).dot(xgapref[:-self.nx]),
            -self.zeta*self.QN.dot(xgapref[-self.nx:]),
            numpy.zeros(self.h*self.nu)
        ])

        return p_matrix, q_vector

    def get_assumed_state(self):
        """Returns the assumed state. """
        return self.assumed_x[:]

    def get_instantaneous_acceleration(self):
        """Returns the optimal acceleration for the current time instant.
        Returns 0 if none available. """
        try:
            return self.get_input_trajectory()[0]
        except TypeError:
            return 0

    def get_input_trajectory(self):
        """Returns the optimal input trajectory computed by the MPC. Returns
        an array of zeros if there is no optimal input trajectory available. """
        try:
            z_copy = self.z.value[:]
            trajectory = numpy.array(z_copy[(self.h + 1)*self.nx:]).flatten()
            return trajectory
        except TypeError:
            return numpy.zeros(self.h*self.nu)

    @staticmethod
    def interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c


def main():
    """
    horizon = 10
    delta_t = 0.5
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.5
    s0 = 0.
    v0 = 1.
    Q = numpy.array([1, 0, 0, 1]).reshape(2, 2)
    QN = Q
    R = numpy.array([1]) * 0.1
    v_min = 0.
    v_max = 4.
    s_min = 0.
    s_max = 1000000
    acc_min = -2.
    acc_max = 2.
    truck_length = 0.5*0
    safety_distance = 0.3
    timegap_scale = 2
    timegap = timegap_scale * delta_t

    # Test optimal speed profile.
    opt_step = 20
    opt_v = 1.
    pos = opt_v * delta_t * numpy.arange(opt_step)
    vel = opt_v * numpy.ones(opt_step)
    vopt = speed.Speed(pos, vel)

    # Test preceding vehicle state.
    prec_vel = opt_v*numpy.ones(horizon*2)
    prec_pos = numpy.hstack([numpy.zeros(horizon - timegap_scale),
                           numpy.arange(1, horizon + timegap_scale + 1)])
    prec_x = TruckMPC.interleave_vectors(prec_vel, prec_pos)

    # Create TruckMPC instance.
    x0 = numpy.array([s0, v0])
    x_min = numpy.array([v_min, s_min])
    x_max = numpy.array([v_max, s_max])
    u_min = numpy.array([acc_min])
    u_max = numpy.array([acc_max])

    tr = TruckMPC(Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                  safety_distance, timegap, x0=x0, xmin=x_min, xmax=x_max,
                  umin=u_min, umax=u_max, QN=QN)

    tr.set_new_x0(x0)
    tr.compute_optimal_trajectories(vopt, prec_x)
    optx = tr.get_assumed_state()
    optu = tr.get_input_trajectory()
    print(optx.shape)
    print(optu.shape)

    print_stuff = True
    if print_stuff:
        for i in range(horizon*2):
            s = 'i = {}: v = {:.1f}, s = {:.1f}'.format(
                i, optx[i*2], optx[i*2 + 1])
            if i > horizon:
                s += ', a = {:.2f}'.format(optu[i % horizon])

            print(s)

    print(tr.get_instantaneous_acceleration())
    """

if __name__ == '__main__':
    main()

