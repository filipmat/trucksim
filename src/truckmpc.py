import numpy
import scipy.sparse as sparse
import cvxpy

import speed

"""
Position: either position along path, or position measured relative the 
position of the leader vehicle at the current time instant. 
"""

# TODO: timegap cost and safety constraint for follower mpcs

class TruckMPC(object):

    def __init__(self, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, QN=None,
                 vehicle_id=None):

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

        if vehicle_id is None:
            self.vehicle_id = 'NO_ID'
        else:
            self.vehicle_id = vehicle_id

        self.nx = self.Ad.shape[0]
        self.nu = self.Bd.shape[1]

        self.inf = 1000000

        self.is_leader = False

        self.status = 'OK'

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


        self.x = cvxpy.Variable((self.h + 1) * self.nx)
        self.u = cvxpy.Variable(self.h * self.nu)

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
            self.status = 'OK'
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(prob.status))
            self.status = 'Could not solve MPC'

    def update_assumed_state(self):
        """Updates the assumed state. The length is two horizons. The first part
        are the previous states, the second part starts with the current state
        and then contains the optimal state from the MPC solution. """
        try:
            self.assumed_x[self.h*self.nx:] = \
                self.x.value[:self.h*self.nx].flatten()
        except TypeError:
            self.assumed_x[self.h*self.nx:] = self.get_backup_predicted_state()
            self.status = 'No MPC optimal. '

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
        ref_state_cost = self.get_state_reference_cost()
        ref_input_cost = self.get_input_reference_cost()

        cost = ref_state_cost + ref_input_cost

        if not self.is_leader:
            gap_cost = self.get_timegap_cost()
            cost = cost + gap_cost

        return cost

    def get_mpc_constraints(self):
        """Returns the constraints for the MPC problem. """
        dynamics_constraints = self.get_dynamics_constraints()
        input_constraints = self.get_input_constraints()
        state_constraints = self.get_state_constraints()

        constraints = dynamics_constraints + input_constraints + \
                      state_constraints

        if not self.is_leader:
            safety_constraints = self.get_safety_constraints()
            constraints = constraints + safety_constraints

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
            u_copy = self.u.value[:]
            trajectory = numpy.array(u_copy).flatten()
            return trajectory
        except TypeError:
            return numpy.zeros(self.h*self.nu)

    @staticmethod
    def interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c

    def get_state_reference_cost(self):
        PQ = numpy.kron(numpy.eye(self.h + 1), (1 - self.zeta) * self.Q)
        PQ_ch = numpy.linalg.cholesky(PQ)

        cost = cvxpy.sum_entries(cvxpy.square(PQ_ch * (self.x - self.xref)))

        return cost

    def get_input_reference_cost(self):
        PR = numpy.kron(numpy.eye(self.h), self.R)
        PR_ch = numpy.linalg.cholesky(PR)

        cost = cvxpy.sum_entries(cvxpy.square(PR_ch * (self.u - self.uref)))

        return cost

    def get_dynamics_constraints(self):
        AA = numpy.kron(numpy.eye(self.h + 1), -numpy.eye(self.nx)) + \
             numpy.kron(numpy.eye(self.h + 1, k=-1), self.Ad)
        BB = numpy.kron(numpy.vstack([numpy.zeros(self.h), numpy.eye(self.h)]),
                        self.Bd)
        xZero = numpy.hstack([-self.x0, numpy.zeros(self.h * self.nx)])

        constraints = [AA * self.x + BB * self.u == xZero]

        return constraints

    def get_input_constraints(self):
        AU = numpy.eye(self.h*self.nu)
        upper = numpy.tile(self.umax, self.h)
        lower = numpy.tile(self.umin, self.h)

        constraints = [AU * self.u <= upper, AU * self.u >= lower]

        return constraints

    def get_state_constraints(self):
        AX = numpy.eye((self.h + 1) * self.nx)

        upper = numpy.tile(self.xmax, self.h + 1)
        lower = numpy.tile(self.xmin, self.h + 1)

        constraints = [AX * self.x <= upper, AX * self.x >= lower]

        return constraints

    def get_timegap_cost(self):
        shift = int(round(self.timegap / self.dt))
        xgapref = self.preceding_x[
                       (self.h - shift)*self.nx:(2*self.h - shift + 1)*self.nx]

        PQ = numpy.kron(numpy.eye(self.h + 1), self.zeta * self.Q)
        PQ_ch = numpy.linalg.cholesky(PQ)

        cost = cvxpy.sum_entries(cvxpy.square(PQ_ch * (self.x - xgapref)))

        return cost

    def get_safety_constraints(self):
        pos = self.preceding_x[
              (self.h - 1) * self.nx:(2 * self.h) * self.nx][1::2]
        posmax = pos - self.truck_length - self.safety_distance
        vmax = numpy.ones(self.h + 1)*self.inf

        statemax = self.interleave_vectors(vmax, posmax)

        AS = numpy.eye((self.h + 1)*self.nx)

        constraints = [AS * self.x <= statemax]

        return constraints


def main():
    pass

if __name__ == '__main__':
    main()
