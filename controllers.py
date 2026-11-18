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

    def __init__(self, v0=30.0, T=1.5, a_max=0.73, b=1.67, delta=4.0, s0=5.0):
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

    def __init__(self, alpha=1.0, tau=2.5, gmin=5, lam=0.1):
        self.alpha = alpha
        self.tau = tau
        self.gmin = gmin
        self.lam = lam

    def cmd_accel(self, sn, vn, sndot):
        return min(1.5, max(-3,self.alpha * (sn - self.tau*vn - self.gmin) + self.lam*sndot))

class SpecialTimeGapController(Controller):
    """
    TG with braking-only damping: lam * sndot is applied only when closing in
    (sndot < 0), as in the WTJ controllers, so it doesn't speed up just because
    the car ahead does. Preset STG.
    """
    name = "STG"

    def __init__(self, alpha=1.0, tau=2.5, gmin=5, lam=2):
        self.alpha = alpha
        self.tau = tau
        self.gmin = gmin
        self.lam = lam

    def cmd_accel(self, sn, vn, sndot):
        return min(1.5, max(-3,self.alpha * (sn - self.tau*vn - self.gmin) + min(0,self.lam*sndot)))


class WTJOriginal(Controller):
    """
    The original WTJ controller: a_star (zero at 30 m/s, larger at low speed)
    clamped to [al, au], the time-gap accelerations for tau_max and tau_min.
    Presets WTJ0 (lam = 0.5) and WTJ1 (lam = 2) in PRESETS.
    """
    name = "WTJOriginal"

    def __init__(self, alpha=1.0, tau_min=2.0, tau_max=3.0, gmin=5, lam=0.5, c=0.1):
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


class WTJSoftLowerBound(WTJOriginal):
    """
    WTJ without the hard lower bound al. Where WTJOriginal picks a_star clamped
    to [al, au], this targets a blend of al (the tau_max time-gap acceleration)
    and a_star,

        a_target = beta * al + (1 - beta) * a_star,

    capped above by au only. So it does not chase every speed-up of the car
    ahead (smoother, less work), while beta still pulls it toward the tau_max
    gap: it closes large gaps and starts braking earlier when inside that gap.
    beta = 0 ignores al entirely. lam = 3 by default (lam = 0.5 crashed on
    wtj_real_drive when closing on a stopping car).
    Presets WTJ2 (beta = 0), WTJ3 (beta = 0.1) and WTJ4 (beta = 0.25) in PRESETS.
    """
    name = "WTJSoftLowerBound"

    def __init__(self, alpha=1.0, tau_min=2.0, tau_max=3.0, gmin=5, lam=3.0, c=0.1, beta=0.25):
        super().__init__(alpha=alpha, tau_min=tau_min, tau_max=tau_max, gmin=gmin, lam=lam, c=c)
        self.beta = beta

    def cmd_accel(self, sn, vn, sndot):
        al = min(1.5, max(-3,self.alpha * (sn - self.tau_max*vn - self.gmin) + min(0,self.lam*sndot)))
        au = min(1.5, max(-3,self.alpha * (sn - self.tau_min*vn - self.gmin) + min(0,self.lam*sndot)))
        if vn <= 0:
            return al

        # WTJ's a_star: zero at 30 m/s, larger at low speed
        a_star = self.c * sqrt(1/(vn + 0.25)) - (self.c * sqrt(1/(30 + 0.25)))
        a_target = self.beta * al + (1 - self.beta) * a_star
        return min(1.5, max(-3,min(a_target, au)))


# Named controllers: name -> (class, parameters that differ from the class
# defaults). A preset is just its class with those parameters; the name is the
# vehicle type in the CSVs and plots, and preset_label() spells out the parameters.
PRESETS = {
    "IDM":  (IDMController, {}),
    "TG":   (TimeGapController, {}),
    "STG":  (SpecialTimeGapController, {}),
    "WTJ0": (WTJOriginal, {}),                       # lam = 0.5
    "WTJ1": (WTJOriginal, {"lam": 2.0}),
    "WTJ2": (WTJSoftLowerBound, {"beta": 0.0}),      # lam = 3
    "WTJ3": (WTJSoftLowerBound, {"beta": 0.1}),      # lam = 3
    "WTJ4": (WTJSoftLowerBound, {"beta": 0.25}),     # lam = 3
}
CONTROLLERS = {name: cls for name, (cls, _) in PRESETS.items()}   # name -> class

# Our controllers: the ones the penetration rate assigns, and that get ghosts
SMART_CONTROLLERS = ("WTJ0", "WTJ1", "WTJ2", "WTJ3", "WTJ4")
# Other controllers --smart may assign, for comparison (no ghosts)
COMPARISON_CONTROLLERS = ("TG", "STG")


def preset_params(name, params=None):
    """Every parameter of preset `name`, with `params` overrides applied, e.g. {'alpha': 1.0, ..., 'beta': 0.1}."""
    cls, preset = PRESETS[name]
    full = {p.name: p.default for p in signature(cls).parameters.values()}
    return {**full, **preset, **(params or {})}


def preset_label(name, params=None):
    """'WTJ3 = WTJSoftLowerBound(alpha=1, tau_min=2, tau_max=3, gmin=5, lam=3, c=0.1, beta=0.1)'."""
    cls = PRESETS[name][0]
    args = ", ".join(f"{k}={v:g}" for k, v in preset_params(name, params).items())
    return f"{name} = {cls.__name__}({args})"


def controller_param_names(name):
    """Parameter names a controller's constructor accepts, e.g. ['alpha', 'tau', ...] for TG."""
    return [p for p in signature(CONTROLLERS[name]).parameters if p != "self"]


def make_controller(name, params=None):
    """
    Build the controller called `name` (a key of CONTROLLERS), overriding any of
    its default parameters with `params`, e.g. make_controller("WTJ3", {"c": 0.2}).
    The preset's parameters (PRESETS) are applied first, then `params`.
    """
    params = params or {}
    unknown = sorted(set(params) - set(controller_param_names(name)))
    if unknown:
        raise ValueError(f"{name} controller has no parameter(s) {unknown}; "
                         f"it takes {controller_param_names(name)}")
    controller = CONTROLLERS[name](**preset_params(name, params))
    controller.name = name   # the preset name ("WTJ3") is the vehicle type, not the class name
    return controller