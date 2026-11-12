from abc import ABC, abstractmethod
from inspect import signature
from math import sqrt

class Controller(ABC):
    name = "?"  # Short label used as the vehicle type in plots and CSVs

    @abstractmethod
    def cmd_accel(self, sn, vn, sndot):
        pass

class IDMController(Controller):
    name = "IDM"

    def __init__(self):
        raise NotImplementedError("TBD")

    def cmd_accel(self, sn, vn, sndot):
        raise NotImplementedError("TBD")

class TimeGapController(Controller):
    name = "TG"

    def __init__(self, alpha=1.0, tau=2.5, gmin=15, lam=0.1):
        self.alpha = alpha
        self.tau = tau
        self.gmin = gmin
        self.lam = lam

    def cmd_accel(self, sn, vn, sndot):
        return min(1.5, max(-3,self.alpha * (sn - self.tau*vn - self.gmin) + self.lam*sndot))

class WTJController(Controller):
    name = "WTJ"

    def __init__(self, alpha=1.0, tau_min=2.0, tau_max=3.0, gmin=15, lam=0.5, c=0.1):
        self.c = c

    def cmd_accel(self, sn, vn, sndot):
        al = min(1.5, max(-3,self.alpha * (sn - self.tau_max*vn - self.gmin) + min(0,self.lam*sndot)))
        au = min(1.5, max(-3,self.alpha * (sn - self.tau_min*vn - self.gmin) + min(0,self.lam*sndot)))
        a_star = 0
        if vn <= 0:
            return al
        
        a_star = self.c * sqrt(1/(vn + 0.25)) - (self.c * sqrt(1/(30 + 0.25)))
        return min(1.5, max(-3,min(max(al, a_star), au)))


# Controllers by name, for picking one from a string (e.g. on the command line)
CONTROLLERS = {cls.name: cls for cls in (TimeGapController, WTJController)}


def controller_param_names(name):
    """Parameter names a controller's constructor accepts, e.g. ['alpha', 'tau', ...] for TG."""
    return [p for p in signature(CONTROLLERS[name]).parameters if p != "self"]


def make_controller(name, params=None):
    """
    Build the controller called `name` (a key of CONTROLLERS), overriding any of
    its default parameters with `params`, e.g. make_controller("WTJ", {"c": 0.2}).
    """
    params = params or {}
    unknown = sorted(set(params) - set(controller_param_names(name)))
    if unknown:
        raise ValueError(f"{name} controller has no parameter(s) {unknown}; "
                         f"it takes {controller_param_names(name)}")
    return CONTROLLERS[name](**params)