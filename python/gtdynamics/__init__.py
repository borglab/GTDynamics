import gtsam

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
