import logging
import sys

from dodal.log import LOGGER as dodal_logger
from dodal.log import (
    DodalLogHandlers,
)

LOGGER = logging.getLogger("I10")
LOGGER.setLevel("DEBUG")
LOGGER.parent = dodal_logger
__logger_handlers: DodalLogHandlers | None = None

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
LOGGER.addHandler(handler)
