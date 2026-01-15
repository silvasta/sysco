from .base_controller import BaseController
import numpy as np

from sysco.schemas import SystemConfig, CostConfig, MPCConfig

system: dict = {
    "A": [[1, 0], [1, 1]],
    "B": [[0], [1]],
    "C": [0, 0],
    "D": 0,
}
cost: dict = {
    "Q": [[1, 0], [0, 1]],
    "R": 1,
}

config = MPCConfig(
    system=SystemConfig(**system),
    cost=CostConfig(**cost),
    horizon=5,
)


class MPC(BaseController[MPCConfig]):
    """Finite horizon, Constraints, ..."""

    horizon: int

    def __init__(self, config: MPCConfig):
        super().__init__(config)

    def solve(self):
        return np.array([0, 0, 0])

    def stage_cost(self):
        pass

    def terminal_cost(self):
        pass

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
