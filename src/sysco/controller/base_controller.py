from ctypes import ArgumentError
from scipy.linalg import solve_discrete_are
import cvxpy as cvx
from enum import Enum
import numpy as np
import control as ct


class BaseController:
    """Foundation of everything,
    minimum requirement: return u
    maximum requirement? that is the task"""

    def __init__(self):
        name = self.__class__.__name__
        print(f"{name} loaded...")

    def get_input(self):
        raise NotImplementedError("Subclass must implement get_input()")

    def hello(self) -> str:
        name = self.__class__.__name__
        mod = self.__module__
        return f"Hello from {name} in module {mod}"


class LQR(BaseController):
    """Controller for infinite horizon
    no constraints
    no disturbance"""

    # TODO: finite horizon LQR? (batch approach or dynamic programming)

    # system params
    A: np.ndarray
    n_x: int
    B: np.ndarray
    n_u: int
    # cost params
    Q: np.ndarray
    R: np.ndarray

    # # current state
    # x_t: np.ndarray

    # set method to calculate dare
    class DARE_Method(Enum):
        scipy = "scipy from control library"
        slycot = "slycot from control library (needs slycot module sg02ad!)"
        optimizer = "NOT WORKING SO FAR! (non-convex) optimizer to solve discrete ARE"
        scipy_direct = "scipy direct from scipy"

    method: DARE_Method

    # results of computation
    # solution of DRE
    P_infinity: np.ndarray
    # gain calculatet with P, used for next input
    F_infinity: np.ndarray

    # NOTE: implement as nicely printable function in any controller
    feasibility: str = "If the system is stabilizable, then there is an input sequence that yields a finite cost"

    def __init__(self, A, B, Q, R, method: DARE_Method = DARE_Method.scipy_direct):
        """Load system params and solve infinite Riccatti to get
        P (something like cost matrix) and
        F (gain matrix) needed to get u (input)"""

        # TODO:  load param and check dimension in more generall class
        # super().__init__()

        self.A = np.atleast_2d(A)
        a_h, a_w = self.A.shape
        if a_h == a_w:
            self.n_x = a_w
        else:
            raise ValueError(f"A should be square, got heigth: {a_h}, width: {a_w}")

        self.B = np.atleast_2d(B)
        b_h, b_w = self.B.shape
        if b_h == a_h:
            self.n_u = b_w
        else:
            raise ValueError(f"A and B doesn't match, heigth A: {a_h}, B: {b_h}")

        self.Q = np.atleast_2d(Q)
        q_h, q_w = self.Q.shape
        if not (q_h == q_w == self.n_x):
            raise ValueError(f"Wrong dimensions for Q: {self.Q.shape}, n_x: {self.n_x}")

        self.R = np.atleast_2d(R)
        r_h, r_w = self.R.shape
        if not (r_h == r_w == self.n_u):
            raise ValueError(f"Wrong dimensions for R: {self.R.shape}, n_u: {self.n_u}")

        # HACK: check stabilizability (or at least controllability)

        # dare method selecor factory
        self.method: LQR.DARE_Method = method
        methods = {
            "scipy": self.dare_control_lib,
            "slycot": self.dare_control_lib,
            "optimizer": self.dare_optimizer,
            "scipy_direct": self.dare_scipy_direct,
        }
        self.P_infinity, self.F_infinity = methods[self.method.name]()

    def dare_control_lib(self):
        """use the method from control lib, that uses scipy
        or use slycot but that needs out of python installed modules"""
        K, S, _ = ct.dlqr(self.A, self.B, self.Q, self.R, method=self.method.name)
        # important! K from control has no - minus sign, added for consistency
        return S, -K

    def dare_scipy_direct(self):
        """use the method from scipy directly"""
        balanced = True  # default, NOTE: test with balanced = False
        P = solve_discrete_are(self.A, self.B, self.Q, self.R, balanced=balanced)
        # load params for shorter equation
        A, B, R = self.A, self.B, self.R
        # get final gain to apply on x to get u
        F = -np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        return P, F

    def dare_optimizer(self):
        """NOT WORKING, idea: approximate Riccatti with optimizer"""
        # TODO: find a way for non-convex OP
        P = cvx.Variable((self.n_x, self.n_x), symmetric=True)
        A, B, Q, R = self.A, self.B, self.Q, self.R
        constraints = []
        constraints += [P >> 0]
        bTPA = B.T @ P @ A
        frac = R + B.T @ P @ B
        constraints += [P == Q + A.T @ P @ A - cvx.matrix_frac(bTPA, frac)]
        objective = 0
        print(cvx.installed_solvers())
        problem = cvx.Problem(cvx.Minimize(objective), constraints)
        problem.solve("CLARABEL")
        print(P.value)
        return 0, 0

    def is_controllable(self) -> bool:
        controllability_matrix = ct.ctrb(self.A, self.B)
        # print(controllability_matrix)
        return self.n_x == np.linalg.matrix_rank(controllability_matrix)

    def get_optimal_cost(self, x_t):
        x_t = np.atleast_2d(x_t).reshape(self.n_x, 1)
        optimal_cost = x_t.T @ self.P_infinity @ x_t
        return optimal_cost

    def get_input(self, *args):
        nx = self.n_x
        if len(args) != 1:
            raise ArgumentError(f"exactly 1 argument mandatory, got {len(args)}")
        if len(*args) != nx:
            raise ArgumentError(f"state dimension = {nx}, but argument is {len(*args)}")

        x_t = np.atleast_2d(args[0]).reshape((self.n_x, 1))
        u = self.F_infinity @ x_t

        # NOTE: handle 1d nested array somehow? necessairy?

        return u


class MPC(BaseController):
    """Controller for finite horizon
    constraints
    disturbance"""

    def __init__(self):
        super().__init__()
        self.A = 1
