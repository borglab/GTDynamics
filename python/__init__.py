import gtsam
from gtdynamics.gtdynamics import *


class GtdKeyFormatter(object):
    def __repr__(self):
        return GtdFormat(self)


class Values(GtdKeyFormatter, gtsam.Values):
    pass


class NonlinearFactorGraph(GtdKeyFormatter, gtsam.NonlinearFactorGraph):
    pass
