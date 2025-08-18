import cvxpy as cvx
import numpy

A = numpy.array([1, -2])

x = cvx.Variable(2)

objective = cvx.Minimize(cvx.sum_squares(A @ x))

constraints = []
constraints += [0 <= x, x <= 1]

prob = cvx.Problem(objective, constraints)

result = prob.solve()

print(x.value)
