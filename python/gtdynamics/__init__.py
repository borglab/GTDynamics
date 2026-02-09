# Python needs to know about gtsam base classes before it can import module classes
# Else will throw cryptic "referenced unknown base type" error.
# Load gtsam with RTLD_GLOBAL so that libgtsam's symbols (and pybind11 type
# registrations) from the gtsam-develop wheel are visible to the gtdynamics
# native extension, which links against the same library but doesn't bundle it.
import sys
import os

_old_dlopen_flags = sys.getdlopenflags()
sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_NOW)
import gtsam
sys.setdlopenflags(_old_dlopen_flags)

from gtdynamics.gtdynamics import *

from . import sim


class _GtdKeyFormatter(object):
    """Private class to format Values and NonlinearFactorGraph keys correctly."""
    def __repr__(self):
        return GtdFormat(self)


class Values(_GtdKeyFormatter, gtsam.Values):
    pass


class NonlinearFactorGraph(_GtdKeyFormatter, gtsam.NonlinearFactorGraph):
    pass
