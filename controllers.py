from abc import ABC, abstractmethod
from math import sqrt

class Controller(ABC):
    @abstractmethod
    def cmd_accel(self, sn, vn, sndot):
        pass

class IDMController(Controller):
    def __init__(self):
        raise NotImplementedError("TBD")

    def cmd_accel(self, sn, vn, sndot):
        raise NotImplementedError("TBD")

class TimeGapController(Controller):
    def __init__(self, alpha=1.0, tau=2.5, gmin=15, lam=.5):
        self.alpha = alpha
        self.tau = tau
        self.gmin = gmin
        self.lam = lam

    def cmd_accel(self, sn, vn, sndot):
        return self.alpha * (sn - self.tau*vn - self.gmin) + self.lam*sndot

class WTJController(Controller):
    def __init__(self, alpha=1.0, tau_min=2.0, tau_max=3.0, gmin=15, lam=0.5, c=0.1):
        self.time_gap_controller_max = TimeGapController(alpha, tau_max, gmin, lam)
        self.time_gap_controller_min = TimeGapController(alpha, tau_min, gmin, lam)
        self.c = c

    def cmd_accel(self, sn, vn, sndot):
        al = self.time_gap_controller_max.cmd_accel(sn, vn, sndot)
        au = self.time_gap_controller_min.cmd_accel(sn, vn, sndot)
        a_star = 0
        if vn <= 0:
            return al
        
        a_star = self.c * sqrt(1/(vn + 0.25)) - (self.c * sqrt(1/(30 + 0.25)))
        return min(max(al, a_star), au)