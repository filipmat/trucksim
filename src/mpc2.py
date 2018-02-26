import cvxpy
import numpy
import scipy.sparse as sparse


horizon = 10
delta_t = 0.5
x0 = numpy.array([0, 0])
v_value = 1.
x_min = numpy.array([0, 0])
x_max = numpy.array([3, 2 * v_value * horizon])
u_min = numpy.array([-2])
u_max = numpy.array([2])
Q = numpy.matrix([[1, 0], [0, 1]])
R = numpy.array([1])

A = numpy.matrix([[1, 0], [delta_t, 1]])
B = numpy.matrix([delta_t, 0])

xdim = A.shape[0]
udim = B.shape[0]

v_ref = v_value * numpy.ones(horizon)
s_ref = v_value * numpy.arange(horizon)

x_ref = numpy.vstack([v_ref, s_ref])

z = cvxpy.Variable(horizon * (xdim + udim))

P = sparse.block_diag([sparse.kron(sparse.eye(horizon), Q),
                       sparse.kron(sparse.eye(horizon), R)])



cost = cvxpy.quad_form(z, P)
objective = cvxpy.Minimize(cost)
