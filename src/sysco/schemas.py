from typing import TYPE_CHECKING, Annotated, Any, Generic, Optional, TypeVar

import numpy as np
from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)


def to_matrix(v: Any) -> np.ndarray:
    """Coerces input into a 2D numpy array."""
    arr = np.array(v, dtype=float)
    if arr.ndim == 0:  # Scalar case: 5.0 -> [[5.0]]
        return arr.reshape((1, 1))
    if arr.ndim == 1:  # Vector case: [1, 2] -> [[1, 2]]
        return arr.reshape((1, -1))
    return arr


# Use this everywhere in your schemas
StrictArray = Annotated[np.ndarray, BeforeValidator(to_matrix)]


class SystemConfig(BaseModel):
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True,
        # This helper tells Pydantic how to treat np.ndarray during export
        json_encoders={np.ndarray: lambda v: v.tolist()},
    )

    A: StrictArray
    B: StrictArray
    C: StrictArray
    D: StrictArray
    dt: Optional[float] = None

    @property
    def n_states(self) -> int:
        return self.A.shape[0]

    @property
    def n_inputs(self) -> int:
        return self.B.shape[1]

    @property
    def n_outputs(self) -> int:
        return self.C.shape[0]

    @model_validator(mode="after")
    def validate_dimensions(self) -> "SystemConfig":
        """
        Check Linear State Space dimensions:
        x_dot = Ax + Bu
        y     = Cx + Du
        """
        nx: int = self.n_states
        if self.A.shape[1] != nx:
            raise ValueError(f"A must be square (nx, nx). Got {self.A.shape}")
        if self.B.shape[0] != nx:
            raise ValueError(f"B rows must match A rows ({nx}). Got {self.B.shape[0]}")
        if self.C.shape[1] != nx:
            raise ValueError(f"C cols must match A rows ({nx}). Got {self.C.shape[1]}")
        nu: int = self.n_inputs
        ny: int = self.n_outputs
        if self.D.shape != (ny, nu):
            raise ValueError(f"D must be (ny, nu) -> ({ny}, {nu}). Got {self.D.shape}")
        return self


class CostConfig(BaseModel):
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True,
        # This helper tells Pydantic how to treat np.ndarray during export
        json_encoders={np.ndarray: lambda v: v.tolist()},
    )

    Q: StrictArray
    R: StrictArray

    @field_validator("Q", "R")
    @classmethod
    def check_symmetry(cls, v: np.ndarray):
        if not np.allclose(v, v.T):  # WARN: symetric or square?
            raise ValueError("Matrix must be symmetric")
        return v

    @field_validator("Q")
    @classmethod
    def check_psd(cls, v: np.ndarray):
        # Check if Positive Semi-Definite (eigenvalues >= 0)
        eigenvalues = np.linalg.eigvals(v)
        if np.any(eigenvalues < -1e-9):  # Small epsilon for float precision
            raise ValueError("Q must be positive semi-definite")
        return v

    @field_validator("R")
    @classmethod
    def check_pd(cls, v: np.ndarray):
        # Check if Positive Definite (eigenvalues > 0)
        eigenvalues = np.linalg.eigvals(v)
        if np.any(eigenvalues <= 0):
            raise ValueError("R must be positive definite (strictly greater than 0)")
        return v


class MPCConfig(BaseModel):
    system: SystemConfig
    cost: CostConfig
    horizon: int = Field(gt=0)

    def __getattr__(self, name: str) -> Any:
        # Priority 1: System matrices/dims
        if hasattr(self.system, name):
            return getattr(self.system, name)
        # Priority 2: Cost matrices
        if hasattr(self.cost, name):
            return getattr(self.cost, name)
        raise AttributeError(f"MPCConfig has no attribute {name}")

    # TEST:
    # Optional: for IDE autocompletion/type checkers
    # if TYPE_CHECKING:
    #     A: np.ndarray; B: np.ndarray; C: np.ndarray; D: np.ndarray
    #     Q: np.ndarray; R: np.ndarray
    #     n_states: int; n_inputs: int

    @model_validator(mode="after")
    def validate_interface(self) -> "MPCConfig":
        # Cross-model validation
        if self.cost.Q.shape[0] != self.system.n_states:
            raise ValueError("Q dimension mismatch with system states")
        if self.cost.R.shape[0] != self.system.n_inputs:
            raise ValueError("R dimension mismatch with system inputs")
        return self
