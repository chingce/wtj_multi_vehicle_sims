from abc import ABC, abstractmethod
from inspect import signature
from math import sqrt

class Controller(ABC):
    name = "?"  # Short label used as the vehicle type in plots and CSVs

    @abstractmethod
    def cmd_accel(self, sn, vn, sndot):
        pass

class IDMController(Controller):
    """
    Intelligent Driver Model (Treiber, Hennecke & Helbing, 2000).

        a = a_max * [1 - (v / v0)^delta - (s* / s)^2]
        s* = s0 + max(0, v*T + v*dv / (2*sqrt(a_max*b)))

    with s the gap to the leader, v the ego speed, and dv = v - v_lead the
    approach rate (positive when closing in). Defaults are the typical values
    from https://en.wikipedia.org/wiki/Intelligent_driver_model.

    v0:    desired speed (m/s)
    T:     safe time headway (s)
    a_max: maximum acceleration (m/s^2)
    b:     comfortable deceleration (m/s^2, positive)
    delta: acceleration exponent
    s0:    minimum gap / jam distance (m)
    """
    name = "IDM"

    def __init__(self, v0=30.0, T=1.5, a_max=0.73, b=1.67, delta=4.0, s0=2.0):
        self.v0 = v0
        self.T = T
        self.a_max = a_max
        self.b = b
        self.delta = delta
        self.s0 = s0

    def cmd_accel(self, sn, vn, sndot):
        v = max(vn, 0.0)   # a fractional delta can't take a negative base
        dv = -sndot        # sndot is v_lead - v_ego; IDM wants v_ego - v_lead
        # The max(0, .) keeps s* from going below s0 when the leader pulls away
        # fast, which would otherwise make the interaction term brake harder
        # (it's squared). Treiber & Kesting use the same guard.
        s_star = self.s0 + max(0.0, v * self.T + v * dv / (2 * sqrt(self.a_max * self.b)))
        s = max(sn, 1e-3)  # gap of zero or less (collision) -> maximal braking, not a crash
        return self.a_max * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2)

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
        self.alpha = alpha
        self.tau_min = tau_min
        self.tau_max = tau_max
        self.gmin = gmin
        self.lam = lam
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
CONTROLLERS = {cls.name: cls for cls in (TimeGapController, WTJController, IDMController)}


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