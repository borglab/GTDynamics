import gtsam
import gtdynamics as gtd

class CdprController:
    def __init__(self, cdpr, pdes=[], dt=0.01):
        self.cdpr = cdpr
        self.pdes = pdes
        self.dt = dt

    def update(self, values, t):
        tau = gtsam.Values()
        gtd.InsertTorqueDouble(tau, 0, t, 1.)
        gtd.InsertTorqueDouble(tau, 1, t, 1.)
        gtd.InsertTorqueDouble(tau, 2, t, 0.)
        gtd.InsertTorqueDouble(tau, 3, t, 0.)
        return tau