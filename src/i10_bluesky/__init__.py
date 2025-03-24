"""Top level API.

.. data:: __version__
    :type: str

    Version number as calculated by https://github.com/pypa/setuptools_scm
"""

from . import plans
from ._version import __version__
from .plans import utils

__all__ = ["__version__", "plans", "utils"]
