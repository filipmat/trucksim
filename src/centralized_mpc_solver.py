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
        self.R = R
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

        v_slack_cost_factor = 100
        self.v_slack = cvxpy.Variable((self.h + 1) * self.n)
        self.v_slack_P = numpy.eye((self.h + 1) * self.n)*v_slack_cost_factor

        safety_slack_cost_factor = 100
        self.safety_slack = cvxpy.Variable((self.h + 1) * (self.n - 1))
        self.safety_slack_P = numpy.eye((self.h + 1) * self.n)*safety_slack_cost_factor

        state_constraints_lower = self._get_lower_state_constraints()
        state_constraints_upper = self._get_upper_state_constraints()
        input_constraints_upper, input_constraints_lower = self._get_input_constraints()
        x0_constraints = self._get_x0_constraints(numpy.zeros(self.nx * self.n))
        dynamics_constraints = self._get_dynamics_constraints()
        safety_constraints = self._get_safety_constraints()

        self.prob.contraints = state_constraints_lower + state_constraints_upper + \
                               input_constraints_lower + input_constraints_upper + \
                               x0_constraints + dynamics_constraints + \
                               safety_constraints

        # TODO: pre-compute matrices.

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
        self.prob.constraints[4] = self._get_x0_constraints(x0s)

    def _get_references(self, vopt, x0s):
        """Returns the state and input references obtained from current positions and optimal
        speed profile. """
        # TODO: implement.
        xref = numpy.zeros((self.h + 1) * self.nx * self.n)
        uref = numpy.zeros(self.h * self.nu * self.n)

        return xref, uref

    def _get_cost(self, xref, uref):
        """Returns the cost for the MPC problem. """
        # TODO: implement.

        return cvxpy.Minimize(1)

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
        # position - preceding_position - slack_variable < - truck_length - safety_distance
        if self.n == 1:
            return []

        constraint = [self.x[(self.h + 1) * self.nx + 1::2] -
                      self.x[1:(self.h + 1) * self.nx * (self.n - 1) + 1:2] - self.safety_slack <
                      - self.truck_length - self.safety_distance]

        return constraint

    def get_instantaneous_accelerations(self):
        """Returns a list of accelerations containing the first control input for each vehicle. """
        # TODO: implement.
        return numpy.zeros(self.h * self.n)


def main():
    vehicle_amount = 1
    horizon = 5
    delta_t = 0.1
    Ad = numpy.matrix([[1., 0.], [delta_t, 1.]])
    Bd = numpy.matrix([[delta_t], [0.]])
    zeta = 0.7
    Q = numpy.array([1, 0, 0, 0.5]).reshape(2, 2)  # State tracking.
    R = numpy.array([1]) * 0.1  # Input tracking.
    velocity_min = 0.
    velocity_max = 2.
    position_min = -100000.
    position_max = 1000000.
    acceleration_min = -0.5
    acceleration_max = 0.5
    truck_length = 0.2
    safety_distance = 0.1
    timegap = 1.

    xmin = numpy.array([velocity_min, position_min])
    xmax = numpy.array([velocity_max, position_max])
    umin = numpy.array([acceleration_min])
    umax = numpy.array([acceleration_max])

    mpc = MPC(vehicle_amount, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance,
              timegap, xmin=xmin, xmax=xmax, umin=umin, umax=umax)

if __name__ == '__main__':
    main()








