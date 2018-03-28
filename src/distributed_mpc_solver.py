"""
Solver for a MPC problem where the vehicle, if it is a follower, also uses trajectories from
the preceding vehicle in cost and constraints.
"""


import numpy
import cvxpy
import scipy.sparse as sparse


class MPC(object):

    def __init__(self, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, is_leader=False):

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

        self.x = cvxpy.Variable((self.h + 1) * self.nx)
        self.u = cvxpy.Variable(self.h * self.nu)

        v_slack_cost_factor = 100
        self.v_slack = cvxpy.Variable(self.h + 1)
        self.v_slack_P = numpy.eye(self.h + 1)*v_slack_cost_factor

        self.inf = 1000000  # "Infinity" used when there are no limits given.

        self.is_leader = is_leader

        self.status = 'OK'  # Status of solver.

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

        # Reference trajectories.
        self.xref = numpy.zeros((self.h + 1)*self.nx)
        self.uref = numpy.zeros(self.h*self.nu)
        self.xgapref = numpy.zeros((self.h + 1) * self.nx)

        # Problem.
        self.prob = cvxpy.Problem(cvxpy.Minimize(1))

        self.SAFETY_CONSTRAINTS_ACTIVE = True   # TODO: remove.

        # Problem constraints.
        state_constraint_lower = self._get_lower_state_constraints()    # Fixed
        state_constraint_upper = self._get_upper_state_constraints()    # Fixed
        input_constraint_lower, input_constraint_upper = self._get_input_constraints()    # Fixed.
        dynamics_constraints1 = self._get_dynamics_constraints_part_one()       # Update during mpc.
        dynamics_constraints2 = self._get_dynamics_constraints_part_two()       # Fixed.

        self.prob.constraints = state_constraint_lower
        self.prob.constraints += state_constraint_upper
        self.prob.constraints += input_constraint_lower
        self.prob.constraints += input_constraint_upper
        self.prob.constraints += dynamics_constraints1  # Update during mpc.
        self.prob.constraints += dynamics_constraints2
        if not self.is_leader and self.SAFETY_CONSTRAINTS_ACTIVE:
            # Update during mpc.
            self.prob.constraints += self._get_safety_constraints(0, [0], [0])

        # Pre-compute matrices used for cost calculation in order to speed up computation.
        self.state_P = sparse.kron(sparse.eye(self.h + 1), (1 - self.zeta)*self.Q)
        self.input_P = sparse.kron(sparse.eye(self.h), self.R)
        self.timegap_P = sparse.kron(sparse.eye(self.h + 1), self.zeta*self.Q)

        self.state_Q_diag = sparse.kron(sparse.eye(self.h + 1), -(1 - self.zeta)*self.Q)
        self.timegap_Q_diag = sparse.kron(sparse.eye(self.h + 1), -self.zeta * self.Q)
        self.R_diag = sparse.kron(sparse.eye(self.h), -self.R)

        self.timegap_shift = int(round(self.timegap / self.dt))

    def _get_lower_state_constraints(self):
        """Returns the lower constraints for the states. Called on initialization. """
        AX = sparse.eye((self.h + 1) * self.nx)
        lower = numpy.tile(self.xmin, self.h + 1)

        return [AX * self.x >= lower]

    def _get_upper_state_constraints(self):
        """Returns the upper constraints for the states. Includes the slack variable for velocity
        limitation. Called on initialization. """
        AX = sparse.kron(sparse.eye(self.h + 1), [1, 0])
        upper = numpy.tile(self.xmax[0], self.h + 1)

        return [AX*self.x - self.v_slack <= upper]

    def _get_input_constraints(self):
        """Returns the constraints corrseponding to input limits. Called on initialization. """
        AU = numpy.eye(self.h*self.nu)
        upper = numpy.tile(self.umax, self.h)
        lower = numpy.tile(self.umin, self.h)

        return [AU * self.u <= upper], [AU * self.u >= lower]

    def _get_dynamics_constraints_part_one(self):
        """Returns the constraints specifying that x(0) = x0. Called each iteration. """
        AA = sparse.hstack([sparse.eye(self.nx), sparse.csc_matrix((self.nx, self.h*self.nx))])

        constraint = [AA * self.x == self.x0]

        return constraint

    def _get_dynamics_constraints_part_two(self):
        """Returns the constraints for x(k+1) = Ax(k) + Bu(k). Called on initialization. """
        AA = sparse.kron(sparse.hstack([sparse.eye(self.h), sparse.csc_matrix((self.h, 1))]),
                         self.Ad) + \
             sparse.kron(sparse.hstack([sparse.csc_matrix((self.h, 1)), sparse.eye(self.h)]),
                         -sparse.eye(self.nx))

        BB = sparse.kron(sparse.eye(self.h), self.Bd)

        constraint = [AA * self.x + BB * self.u == 0]

        return constraint

    def _get_safety_constraints(self, current_time, preceding_timestamps, preceding_positions):
        """Returns the constraints for keeping a safety distance. Called each iteration. """
        target_time = current_time - self.dt + numpy.arange(self.h + 1) * self.dt
        preceding_pos = numpy.interp(target_time, preceding_timestamps, preceding_positions)

        AX = sparse.kron(sparse.eye(self.h + 1), [0, 1])

        posmax = preceding_pos - self.truck_length - self.safety_distance

        constraints = [AX * self.x < posmax]

        return constraints

    def solve_mpc(self, vopt, x0, current_time=None, preceding_timestamps=None,
                  preceding_velocities=None, preceding_positions=None):
        """Solves the MPC problem.
        Computes reference trajectories from optimal speed profile and
        current state. If the vehicle is a follower it also computes the state reference from the
        timegap. Updates the constraints and cost and solves the problem. """
        self.x0 = x0
        self._compute_references(self.x0[1], vopt)

        if not (self.is_leader or
                current_time is None or
                preceding_timestamps is None or
                preceding_velocities is None or
                preceding_positions is None):
            self._compute_xgap_ref(current_time, preceding_timestamps, preceding_velocities,
                                   preceding_positions)
            self._update_safety_constraints(current_time, preceding_timestamps,
                                            preceding_positions)

        self._update_dynamics_constraints()

        cost = self._get_mpc_cost()
        self.prob.objective = cvxpy.Minimize(cost)

        try:
            self.prob.solve(solver='ECOS')
            self.status = 'OK'
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(self.prob.status))
            self.status = 'Could not solve MPC'

    def _update_dynamics_constraints(self):
        """Updates the first dynamics constraint x(0) = x0. """
        self.prob.constraints[4] = self._get_dynamics_constraints_part_one()[0]

    def _update_safety_constraints(self, current_time, preceding_timestamps, preceding_positions):
        """Updates the safety constraint. """
        if not self.is_leader and self.SAFETY_CONSTRAINTS_ACTIVE:
            self.prob.constraints[6] = self._get_safety_constraints(current_time,
                                                                    preceding_timestamps,
                                                                    preceding_positions)[0]

    def _compute_references(self, s0, v_opt):
        """Computes the different reference signals. """
        pos_ref = self._get_pos_ref(s0, v_opt)
        vel_ref = self._compute_vel_ref(v_opt, pos_ref)
        acc_ref = self._compute_acc_ref(vel_ref)

        self.xref = self._interleave_vectors(vel_ref, pos_ref)
        self.uref = acc_ref[:]

    def _get_pos_ref(self, s0, v_opt):
        """Computes the position reference trajectory. """
        pos_ref = numpy.zeros(self.h + 1)
        pos_ref[0] = s0

        for i in range(1, self.h + 1):
            pos_ref[i] = pos_ref[i - 1] + self.dt*v_opt.get_speed_at(pos_ref[i - 1])

        return pos_ref

    def _compute_vel_ref(self, v_opt, pos_ref):
        """Computes the velocity reference trajectory. """
        vel_ref = v_opt.get_speed_at(pos_ref)

        return vel_ref

    def _compute_acc_ref(self, vel_ref):
        """Computes the acceleration reference trajectory. """
        acc_ref = (vel_ref[1:] - vel_ref[:-1])/self.dt

        return acc_ref

    def _get_mpc_cost(self):
        """Returns the mpc cost for the current iteration. """
        cost = 0.5*cvxpy.quad_form(self.x, self.state_P) + \
               self.state_Q_diag.dot(self.xref)*self.x + \
               0.5*cvxpy.quad_form(self.u, self.input_P) + self.R_diag.dot(self.uref)*self.u + \
               cvxpy.quad_form(self.v_slack, self.v_slack_P)

        if not self.is_leader:
            timegap_q = self.timegap_Q_diag.dot(self.xgapref)

            cost = cost + 0.5*cvxpy.quad_form(self.x, self.timegap_P) + timegap_q*self.x

        return cost

    def _compute_xgap_ref(self, current_time, timestamps, velocities, positions):
        """Returns the reference state for tracking the timegap of the preceding vehicle. """
        target_time = current_time - self.timegap + numpy.arange(self.h + 1)*self.dt
        gap_vel = numpy.interp(target_time, timestamps, velocities)
        gap_pos = numpy.interp(target_time, timestamps, positions)

        self.xgapref = self._interleave_vectors(gap_vel, gap_pos)

    def get_instantaneous_acceleration(self):
        """Returns the optimal acceleration for the current time instant.
        Returns 0 if none available. """
        try:
            return self.get_input_trajectory()[0]
        except TypeError:
            return 0

    def get_input_trajectory(self):
        """Returns the optimal input trajectory computed by the MPC. Returns an array of zeros if
        there is no optimal input trajectory available. """
        try:
            trajectory = numpy.array(self.u.value[:]).flatten()
            return trajectory
        except TypeError:
            print('MPC returning acc = 0')
            return numpy.zeros(self.h*self.nu)

    def get_predicted_states(self):
        """Returns the predicted velocity and position from the MPC, starting at x0. If there is
        no trajectories available a backup state is returned, which assumes zero acceleration. """
        try:
            return numpy.squeeze(numpy.asarray(self.x.value[0::2].flatten())), \
                   numpy.squeeze(numpy.asarray(self.x.value[1::2].flatten()))
        except TypeError:
            return self._get_backup_predicted_state()

    def _get_backup_predicted_state(self):
        """Returns a predicted state trajectory over one horizon. Used if there
        is no optimal trajectory available from the MPC solver. """
        vel = numpy.ones(self.h + 1)*self.x0[0]
        pos = self.x0[1] + self.x0[0]*self.dt*numpy.arange(self.h + 1)

        return vel, pos

    @staticmethod
    def _interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)























