from loguru import logger

from sysco._core import hello_from_bin

from .controller import LQR as LQR
from .controller import MPC as MPC

logger.debug(f"Importing {__name__}")


def hello_from_rust() -> str:
    return hello_from_bin()
