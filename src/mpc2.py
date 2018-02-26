import cvxpy
import numpy
import scipy.sparse as sparse


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
R = numpy.array([1])*0.1

Ad = numpy.matrix([[1, 0], [delta_t, 1]])
Bd = numpy.matrix([[delta_t], [0]])

xdim = Ad.shape[0]
udim = Bd.shape[1]

v_ref = v_value * numpy.ones(horizon)
s_ref = v_value * delta_t * numpy.arange(horizon)

x_ref = numpy.vstack([v_ref, s_ref]).reshape((-1,), order='F')
xN_ref = numpy.array([x_ref[-2], x_ref[-1] + v_value*delta_t])

z = cvxpy.Variable((horizon + 1)*xdim + horizon*udim)

P = sparse.block_diag([sparse.kron(sparse.eye(horizon), Q),
                       QN,
                       sparse.kron(sparse.eye(horizon), R)])

q = numpy.hstack([numpy.kron(numpy.eye(horizon), -Q).dot(x_ref),
                  numpy.array([-QN.dot(xN_ref)]),
                  numpy.zeros((1, horizon*udim))])

Ax = sparse.kron(sparse.eye(horizon + 1), -sparse.eye(xdim)) + \
    sparse.kron(sparse.eye(horizon + 1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, horizon)),
                                sparse.eye(horizon)]),
                 Bd)

Aeq = sparse.hstack([Ax, Bu])
Aineq = sparse.eye((horizon + 1)*xdim + horizon*udim)
A = sparse.vstack([Aeq, Aineq])

upper = numpy.hstack([-x0,
                      numpy.zeros(horizon*xdim),
                      numpy.tile(x_max, horizon + 1),
                      numpy.tile(u_max, horizon)])

lower = numpy.hstack([-x0,
                      numpy.zeros(horizon*xdim),
                      numpy.tile(x_min, horizon + 1),
                      numpy.tile(u_min, horizon)])

cost = 0.5*cvxpy.quad_form(z, P) + q*z
objective = cvxpy.Minimize(cost)
constraints = [A*z <= upper, A*z >= lower]
prob = cvxpy.Problem(objective, constraints)
optimal = prob.solve()

x_opt = z.value[0:(horizon + 1)*xdim]
u_opt = z.value[(horizon + 1)*xdim:]

for i in range(horizon):
    print('t = {:.1f}: v = {:.1f} ({:.1f}), s = {:.1f} ({:.1f}), a = {:.1f}'.format(
        i*delta_t, x_opt[i*2, 0], x_ref[i*2], x_opt[i*2 + 1, 0], x_ref[i*2 + 1],
        u_opt[i, 0]))