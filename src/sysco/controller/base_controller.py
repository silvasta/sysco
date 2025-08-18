from abc import ABC, abstractmethod


class BaseController(ABC):
    """Define functionality every controller must have"""

    def __init__(self):
        pass

    @abstractmethod
    def get_next_input(self):  # TODO: general return type, maybe property to overwride
        raise NotImplementedError("Subclass must implement get_next_input")

    @property
    def _location_in_code(self) -> str:
        name = self.__class__.__name__
        mod = self.__module__
        return f"Class: '{name}' loaded from: {mod}"
