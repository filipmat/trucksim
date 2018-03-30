"""
Solver for a centralized MPC problem. All vehicle problems are solved at once. Follower vehicles
uses state trajectories from preceding vehicle.
"""

import numpy
import cvxpy
import scipy.sparse as sparse


class MPC(object):

    def __init__(self, vehicle_amount, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None):

        self.n = vehicle_amount
        self.Ad = Ad
        self.Bd = Bd
        self.dt = delta_t
        self.h = horizon
        self.zeta = zeta
        self.Q = Q
        self.truck_length = truck_length
        self.safety_distance = safety_distance
        self.timegap = timegap

        self.inf = 1000000  # "Infinity" used when there are no limits given.

        self.nx = self.Ad.shape[0]
        self.nu = self.Bd.shape[1]

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

        self.prob = cvxpy.Problem(cvxpy.Minimize(1))
        self.x = cvxpy.Variable((self.h + 1) * self.nx * self.n)
        self.u = cvxpy.Variable(self.h * self.nu * self.n)

        self.v_slack_cost_factor = 100
        self.v_slack = cvxpy.Variable((self.h + 1) * self.n)

        self.safety_slack_cost_factor = 10000
        self.safety_slack = cvxpy.Variable((self.h + 1) * (self.n - 1))

        x0_constraints = self._get_x0_constraints(numpy.zeros(self.nx * self.n))
        state_constraints_lower = self._get_lower_state_constraints()
        state_constraints_upper = self._get_upper_state_constraints()
        input_constraints_upper, input_constraints_lower = self._get_input_constraints()
        dynamics_constraints = self._get_dynamics_constraints()
        safety_constraints = self._get_safety_constraints()
        v_slack_constraints = self._get_v_slack_constraints()
        safety_slack_constraints = self._get_safety_slack_constraints()

        self.prob.constraints = x0_constraints
        self.prob.constraints += state_constraints_lower
        self.prob.constraints += state_constraints_upper
        self.prob.constraints += input_constraints_lower
        self.prob.constraints += input_constraints_upper
        self.prob.constraints += dynamics_constraints
        self.prob.constraints += v_slack_constraints

        if self.n > 1:
            self.prob.constraints += safety_constraints
            self.prob.constraints += safety_slack_constraints

        self.v_slack_cost = self._get_v_slack_cost()
        self.safety_slack_cost = self._get_safety_slack_cost()
        self.timegap_cost = self._get_timegap_cost()

        self.state_P = sparse.kron(sparse.eye((self.h + 1) * self.n), (1 - zeta)*Q)
        self.state_Q_diag = sparse.kron(sparse.eye((self.h + 1) * self.n), -(1 - zeta)*Q)

        self.input_P = sparse.kron(sparse.eye(self.h * self.n), R)
        self.R_diag = sparse.kron(sparse.eye(self.h * self.n), -R)

    def solve_mpc(self, vopt, x0s):
        """Solves the MPC problem. """
        self._update_x0_constraints(x0s)

        xref, uref = self._get_references(vopt, x0s)

        cost = self._get_cost(xref, uref)

        self.prob.objective = cvxpy.Minimize(cost)

        try:
            self.prob.solve(solver='ECOS')
        except cvxpy.error.SolverError as e:
            print('Could not solve MPC: {}'.format(e))
            print('status: {}'.format(self.prob.status))

    def _update_x0_constraints(self, x0s):
        """Updates the constraint of the problem corresponding to x0. """
        self.prob.constraints[0] = self._get_x0_constraints(x0s)[0]

    def _get_references(self, vopt, x0s):
        """Returns the state and input references obtained from current positions and optimal
        speed profile. """
        pos_ref = numpy.zeros((self.h + 1) * self.n)

        uref = numpy.zeros(self.h * self.nu * self.n)

        # Position reference is obtained from current position and speed profile.
        for j in range(self.n):
            pos_ref[(self.h + 1) * j] = x0s[j*2 + 1]
            for i in range((self.h + 1) * j + 1, (self.h + 1) * (j + 1)):
                pos_ref[i] = pos_ref[i - 1] + self.dt*vopt.get_speed_at(pos_ref[i - 1])

        # Velocity reference is obtained from the speed profile at the reference positions.
        vel_ref = vopt.get_speed_at(pos_ref)

        # Acceleration reference is obtained from the velocity reference.
        for j in range(self.n):
            uref[self.h * j:self.h * (j + 1)] = \
                (vel_ref[(self.h + 1) * j + 1:(self.h + 1) * (j + 1) + 0] -
                 vel_ref[(self.h + 1) * j + 0:(self.h + 1) * (j + 1) - 1]) / self.dt

        # State reference consists of velocities and positions.
        xref = self._interleave_vectors(vel_ref, pos_ref)

        return xref, uref

    def _get_cost(self, xref, uref):
        """Returns the cost for the MPC problem. """

        state_reference_cost = 0.5 * cvxpy.quad_form(self.x, self.state_P) + \
                               self.state_Q_diag.dot(xref) * self.x

        input_reference_cost = 0.5 * cvxpy.quad_form(self.u, self.input_P) + \
                               self.R_diag.dot(uref) * self.u

        cost = self.v_slack_cost
        cost += state_reference_cost
        cost += input_reference_cost

        if self.n > 1:
            cost += self.safety_slack_cost
            cost += self.timegap_cost

        return cost

    def _get_lower_state_constraints(self):
        """Returns the lower constraints for the states. Called on initialization. """
        AX = sparse.eye((self.h + 1) * self.nx * self.n)
        lower = numpy.tile(self.xmin, (self.h + 1) * self.n)

        constraint = [AX * self.x >= lower]

        return constraint

    def _get_upper_state_constraints(self):
        """Returns the upper constraints for the states. Includes the slack variable for velocity
        limitation. Called on initialization. """
        AX = sparse.kron(sparse.eye((self.h + 1) * self.n), [1, 0])
        upper = numpy.ones((self.h + 1) * self.n)*self.xmax[0]

        constraint = [AX*self.x - self.v_slack <= upper]

        return constraint

    def _get_input_constraints(self):
        """Returns the constraints corrseponding to input limits. Called on initialization. """
        AU = numpy.eye(self.h * self.nu * self.n)
        upper = numpy.tile(self.umax, self.h * self.n)
        lower = numpy.tile(self.umin, self.h * self.n)

        constraint = [AU * self.u <= upper], [AU * self.u >= lower]

        return constraint

    def _get_dynamics_constraints(self):
        """Returns the constraints for x(k+1) = Ax(k) + Bu(k). Called on initialization. """
        a = sparse.kron(sparse.hstack([sparse.eye(self.h), sparse.csc_matrix((self.h, 1))]),
                         self.Ad) + \
             sparse.kron(sparse.hstack([sparse.csc_matrix((self.h, 1)), sparse.eye(self.h)]),
                         -sparse.eye(self.nx))

        AX = sparse.kron(sparse.eye(self.n), a)

        BB = sparse.kron(sparse.eye(self.h * self.n), self.Bd)

        constraint = [AX * self.x + BB * self.u == 0]

        return constraint

    def _get_x0_constraints(self, x0s):
        """Returns the constraints for x(0) = x0 for each vehicle. Called on each iteration. """
        a = sparse.lil_matrix((self.n, (self.h + 1) * self.n))
        for i in range(self.n):
            a[i, (self.h + 1)*i] = 1

        AX = sparse.kron(a, sparse.eye(2))

        constraint = [AX * self.x == x0s]

        return constraint

    def _get_safety_constraints(self):
        """Returns the constraints for keeping safety distance for the follower vehicles. Called
        on initialization. """
        if self.n <= 1:
            return []

        matrix_preceding = sparse.kron(sparse.hstack([sparse.eye((self.h + 1)*(self.n - 1)),
                                                      sparse.csc_matrix((self.h + 1, self.h + 1))]),
                                       [0, -1])
        matrix_follower = sparse.kron(sparse.hstack([sparse.csc_matrix((self.h + 1, self.h + 1)),
                                                     sparse.eye((self.h + 1)*(self.n - 1))]),
                                      [0, 1])

        AX = matrix_preceding + matrix_follower

        constraint = [AX * self.x - self.safety_slack < - self.truck_length - self.safety_distance]

        return constraint

    def _get_v_slack_constraints(self):
        """Returns the constraints that the velocity slack variable is positive. Called on
        initialization. """
        constraint = [self.v_slack > 0]

        return constraint

    def _get_safety_slack_constraints(self):
        """Returns the constraints that the safety distance slack variable is positive. Called on
        initialization. """
        if self.n > 1:
            constraint = [self.safety_slack > 0]
        else:
            constraint = []

        return constraint

    def _get_v_slack_cost(self):
        """Returns the cost function for the velocity slack variable. Called on initialization. """
        v_slack_P = numpy.eye((self.h + 1) * self.n)*self.v_slack_cost_factor

        cost = cvxpy.quad_form(self.v_slack, v_slack_P)

        return cost

    def _get_safety_slack_cost(self):
        """Returns the cost function for the safety distance slack variable. Called on
        initialization. """
        if self.n > 1:
            safety_slack_P = numpy.eye((self.h + 1) * (self.n - 1)) * self.safety_slack_cost_factor

            cost = cvxpy.quad_form(self.safety_slack, safety_slack_P)
        else:
            cost = []

        return cost

    def _get_timegap_cost(self):
        """Returns the cost function for timegap tracking. Called on initialization. """
        if self.n <= 1:
            return []

        matrix_preceding = sparse.kron(sparse.hstack([sparse.eye((self.h + 1)*(self.n - 1)),
                                                      sparse.csc_matrix((self.h + 1, self.h + 1))]),
                                       [0, 1])
        matrix_follower = sparse.kron(sparse.hstack([sparse.csc_matrix((self.h + 1, self.h + 1)),
                                                     sparse.eye((self.h + 1)*(self.n - 1))]),
                                      [0, -1])

        A_preceding_follow = matrix_preceding + matrix_follower

        A_follow_v = sparse.kron(sparse.hstack([sparse.csc_matrix((self.h + 1, self.h + 1)),
                                                     sparse.eye((self.h + 1)*(self.n - 1))]),
                                      [-self.timegap, 0])

        AX = A_preceding_follow + A_follow_v

        P = sparse.kron(sparse.eye((self.h + 1) * (self.n - 1)), self.zeta*self.Q[1, 1])

        timegap_cost = cvxpy.quad_form(AX*self.x, P)

        return timegap_cost

    def get_instantaneous_accelerations(self):
        """Returns a list of accelerations containing the first control input for each vehicle. """
        try:
            u_opt = numpy.squeeze(numpy.array(self.u.value))
            accelerations = u_opt[0::self.h]
        except (TypeError, IndexError) as e:
            accelerations = numpy.zeros(self.n)
            print('MPC returning acc = 0: {}'.format(e))

        return accelerations

    @staticmethod
    def _interleave_vectors(a, b):
        """Returns a numpy array where the values in a and b are interleaved."""
        c = numpy.vstack([a, b]).reshape((-1,), order='F')

        return c

    @staticmethod
    def print_numpy(a):
        s = '['
        for v in a.flatten():
            s += ' {:.3f}'.format(v)
        s += ' ]'

        print(s)
