import json
from abc import abstractmethod
from pathlib import Path
from typing import (
    Any,
    Generic,
    TypeVar,
)

import yaml
from pydantic import BaseModel


T = TypeVar("T", bound=BaseModel)


class BaseController(Generic[T]):
    """Define functionality every controller must have"""

    def __init__(self, config: T):
        self.config = config

    def __getattr__(self, name: str) -> Any:
        """Allows self.A, self.Q, self.n_states directly in the controller."""
        return getattr(self.config, name)

    @abstractmethod
    def solve(self):  # TODO: general return type, maybe property to overwride
        raise NotImplementedError("Subclass must implement get_next_input")

    @classmethod
    def from_file(cls, path: str | Path) -> "BaseController[T]":
        """
        Loads config from JSON or YAML and initializes the controller.
        """
        path = Path(path)
        with open(path, "r") as f:
            if path.suffix in (".yaml", ".yml"):
                data = yaml.safe_load(f)
            else:
                data = json.load(f)

        # This uses the TypeVar T to know which Pydantic model to use
        # Note: In a real implementation, you'd access the __orig_bases__
        # to get the specific Config class.
        config_class = cls.__annotations__["config"]
        return cls(config=config_class(**data))

    def save_config(self, path: str | Path):
        """Serializes the current state (useful for experiment logging)."""
        path = Path(path)
        # model_dump handles the conversion of numpy arrays back to lists for JSON
        with open(path, "w") as f:
            json.dump(self.config.model_dump(mode="json"), f, indent=4)

    @property
    def _location_in_code(self) -> str:
        name = self.__class__.__name__
        mod = self.__module__
        return f"Class: '{name}' loaded from: {mod}"
