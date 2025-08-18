from loguru import logger

from sysco._core import hello_from_bin

from .controller.base_controller import BaseController as BaseController
from .controller.MPC import MPC as MPC


def hello_from_rust() -> str:
    return hello_from_bin()


logger.debug(f"{__name__} import completed")
