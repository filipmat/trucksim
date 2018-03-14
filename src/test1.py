import cvxpy
import numpy
import scipy.sparse as sparse
import speed

def print_numpy(a):
    str = '['
    for v in a.flatten():
        str += ' {:.2f}'.format(v)
    str += ' ]'

    print(str)


def interleave_vectors(a, b):
    """Returns a numpy array where the values in a and b are interleaved."""
    c = numpy.vstack([a, b]).reshape((-1,), order='F')

    return c


dt = 0.5
h = 3
v = 1.
v0 = 1.2
s0 = 0

Q = numpy.array([1, 0, 0, 1]).reshape(2, 2)
R = numpy.array([1])

A = numpy.array([1, 0, dt, 1]).reshape(2, 2)
B = numpy.array([dt, 0]).reshape(2, 1)

nx = A.shape[1]
nu = B.shape[1]

x = cvxpy.Variable((h + 1)*nx)
u = cvxpy.Variable(h*nu)
x0 = numpy.array([v0, s0])

v_ref = v*numpy.ones(h + 1)
s_ref = s0 + v*dt*numpy.arange(h + 1)
x_ref = interleave_vectors(v_ref, s_ref)
u_ref = numpy.zeros(h*nu)

PQ = numpy.kron(numpy.eye(h + 1), Q)
PQ_ch = numpy.linalg.cholesky(PQ)

PR = numpy.kron(numpy.eye(h), R)
PR_ch = numpy.linalg.cholesky(PR)

cost = cvxpy.sum_entries(cvxpy.square(PQ_ch * (x - x_ref))) + \
    cvxpy.sum_entries(cvxpy.square(PR_ch * (u - u_ref)))

AA = numpy.kron(numpy.eye(h + 1), -numpy.eye(nx)) + \
    numpy.kron(numpy.eye(h + 1, k=-1), A)
BB = numpy.kron(numpy.vstack([numpy.zeros(h), numpy.eye(h)]), B)
xZero = numpy.hstack([-x0, numpy.zeros(h*nx)])
print(AA)
print(BB)


constraints = [AA*x + BB*u == xZero]

objective = cvxpy.Minimize(cost)
prob = cvxpy.Problem(objective, constraints)
prob.solve()
print_numpy(numpy.array(x.value).flatten())
print_numpy(numpy.array(u.value).flatten())
print_numpy(x_ref)

vv = speed.Speed()
vv.generate_sin(2, 4, 100, 10)
print(vv)

#
# cost = (x - x_ref).T*P*(x - x_ref)
# z1 = cvxpy.Variable(nx)
# L1 = numpy.linalg.cholesky(Q)
# c1 = cvxpy.sum_entries(L1*z1)
# o1 = cvxpy.Minimize(c1)
# p1 = cvxpy.Problem(o1)
# p1.solve()

# cost = z.T*Q*z
# objective = cvxpy.Minimize(cost)
# prob = cvxpy.Problem(objective)
# prob.solve()











































