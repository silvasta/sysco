from .base_controller import BaseController
import numpy as np


class MPC(BaseController):
    """Finite horizon, Constraints, ..."""

    horizon: int

    def __init__(self):
        pass

    def get_next_input(self):
        return np.ndarray([0, 0, 0])

    # cost

    def stage_cost(self):
        pass

    def terminal_cost(self):
        pass

    # constraints

    def dynamics(self):
        pass

    def state_constraints(self):
        pass

    def input_constraints(self):
        pass

    def terminal_constraints(self):
        pass


class StochasticMPC(MPC):
    pass
