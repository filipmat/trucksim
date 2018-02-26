import cvxpy
import numpy
import scipy.sparse as sparse


# noinspection PyPep8
class MPC(object):

    def __init__(self, Ad, Bd, H, x_min, x_max, u_min, u_max, Q, QN, R, x0,
                 x_ref, xN_ref):
        self.nx = Ad.shape[0]
        self.nu = Bd.shape[1]

        A = self.get_A(Ad, Bd, H)
        lower, upper = self.get_limits(H, x_min, x_max, u_min, u_max, x0)
        P = self.get_P(H, Q, QN, R)
        q = self.get_q(H, Q, QN, x_ref, xN_ref)

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

    def get_P(self, H, Q, QN, R):
        P = sparse.block_diag([sparse.kron(sparse.eye(H), Q),
                               QN,
                               sparse.kron(sparse.eye(H), R)])

        return P

    def get_q(self, H, Q, QN, x_ref, xN_ref):
        q = numpy.hstack([numpy.kron(numpy.eye(H), -Q).dot(x_ref),
                          numpy.array([-QN.dot(xN_ref)]),
                          numpy.zeros((1, H * self.nu))])

        return q


def main():
    pass


if __name__ == '__main__':
    main()