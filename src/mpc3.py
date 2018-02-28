import cvxpy
import numpy
import scipy.sparse as sparse


# noinspection PyPep8
class MPC(object):

    def __init__(self, Ad, Bd, H, x_min, x_max, u_min, u_max, Q, QN, R, x0,
                 x_ref, xN_ref, zeta):
        self.nx = Ad.shape[0]
        self.nu = Bd.shape[1]

        A = self.get_A(Ad, Bd, H)
        lower, upper = self.get_limits(H, x_min, x_max, u_min, u_max, x0)
        P_ref = self.get_reference_P(H, Q, QN, R, zeta)
        P_timegap = self.get_timegap_P(H, Q, QN, zeta)
        q = self.get_q(H, Q, x_ref, QN, xN_ref)


        z = cvxpy.Variable((H + 1) * self.nx + H * self.nu)

        cost = 0.5 * cvxpy.quad_form(z, P_ref) + q * z
        objective = cvxpy.Minimize(cost)
        constraints = [A * z <= upper, A * z >= lower]
        prob = cvxpy.Problem(objective, constraints)
        optimal = prob.solve()

        x_opt = z.value[0:(H + 1) * self.nx]
        u_opt = z.value[(H + 1) * self.nx:]

        for i in range(H):
            print(
            'i = {}: v = {:.1f} ({:.1f}), s = {:.1f} ({:.1f}), a = {:.1f}'.format(
                i, x_opt[i * 2, 0], x_ref[i * 2], x_opt[i * 2 + 1, 0],
                x_ref[i * 2 + 1],
                u_opt[i, 0]))

    def get_A(self, Ad, Bd, H):
        Ax = sparse.kron(sparse.eye(H + 1), -sparse.eye(self.nx)) + \
             sparse.kron(sparse.eye(H + 1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, H)),
                                        sparse.eye(H)]),
                         Bd)

        Aeq = sparse.hstack([Ax, Bu])
        Aineq = sparse.eye((H + 1) * self.nx + H * self.nu)
        A = sparse.vstack([Aeq, Aineq])

        return A

    def get_limits(self, H, x_min, x_max, u_min, u_max, x0):
        upper = numpy.hstack([-x0,
                              numpy.zeros(H * self.nx),
                              numpy.tile(x_max, H + 1),
                              numpy.tile(u_max, H)])

        lower = numpy.hstack([-x0,
                              numpy.zeros(H * self.nx),
                              numpy.tile(x_min, H + 1),
                              numpy.tile(u_min, H)])

        return lower, upper

    def get_reference_P(self, H, Q, QN, R, zeta):
        P = sparse.block_diag([sparse.kron(sparse.eye(H), (1 - zeta)*Q),
                               (1 - zeta)*QN,
                               sparse.kron(sparse.eye(H), R)])

        return P

    def get_timegap_P(self, H, Q, QN, zeta):
        P = sparse.block_diag([sparse.kron(sparse.eye(H), zeta*Q),
                               zeta*QN,
                               numpy.zeros((H*self.nu, H*self.nu))])

        return P



    def get_q(self, H, Q, x_ref, QN = None, xN_ref = None):
        if QN is None:
            QN = numpy.zeros(Q.shape)
        if xN_ref is None:
            QN = numpy.zeros(Q.shape)
            xN_ref = numpy.zeros(Q.shape[0])

        q = numpy.hstack([numpy.kron(numpy.eye(H), -Q).dot(x_ref),
                          numpy.array([-QN.dot(xN_ref)]),
                          numpy.zeros((1, H * self.nu))])

        return q


def main():
    horizon = 20
    delta_t = 0.5
    x0 = numpy.array([1, 0])
    v_value = 1.
    x_min = numpy.array([0, 0])
    x_max = numpy.array([3, 2 * v_value * horizon])
    u_min = numpy.array([-4])
    u_max = numpy.array([4])
    Q = numpy.matrix([[1, 0], [0, 1]])
    QN = numpy.zeros((2, 2))
    R = numpy.array([1]) * 0.1

    Ad = numpy.matrix([[1, 0], [delta_t, 1]])
    Bd = numpy.matrix([[delta_t], [0]])

    xdim = Ad.shape[0]
    udim = Bd.shape[1]

    v_ref = v_value * numpy.ones(horizon)
    s_ref = v_value * delta_t * numpy.arange(horizon)

    x_ref = numpy.vstack([v_ref, s_ref]).reshape((-1,), order='F')
    xN_ref = numpy.array([x_ref[-2], x_ref[-1] + v_value * delta_t])

    mpc = MPC(Ad, Bd, horizon, x_min, x_max, u_min, u_max, Q, QN, R, x0, x_ref,
              xN_ref, 0)


if __name__ == '__main__':
    main()