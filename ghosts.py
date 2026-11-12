"""
Ghost cars: counterfactual copies of WTJ vehicles that run a different controller.

When ghosts are on, every WTJ vehicle gets one ghost per requested controller.
A ghost starts with its host's position and speed and, at every step, follows
the same real vehicle the host follows. Nothing follows the ghost, so it never
affects the rest of the platoon. Comparing ghost and host shows what would have
happened had that one WTJ car run the ghost's controller instead.

Usage:
    from sim import sim
    from ghosts import compare_ghosts

    data, time, platoon = sim(100, 0.2, 360, ghosts=["TG"])
    for result in compare_ghosts(data, platoon, dt=0.05):
        print(result.car, result.ghost_type, result.work_diff)
"""

from dataclasses import dataclass, field

import numpy as np

from controllers import CONTROLLERS, make_controller
from metrics import total_positive_work
from vehicles import Vehicle

HOST_TYPE = "WTJ"           # Only vehicles running this controller get ghosts
DEFAULT_GHOSTS = ("TG",)    # Ghost controllers used when ghosts are turned on without a list


class GhostCar(Vehicle):
    """A copy of `host` running a different controller; see the module docstring."""

    def __init__(self, host: Vehicle, controller, n_steps: int):
        super().__init__(vehicle_id=host.id, controller=controller,
                         mass=host.mass, position=host.position, speed=host.speed,
                         variable_mass=host.variable_mass)
        self.host = host
        # Same layout as one row of the sim output: position, speed, acceleration per step
        self.trajectory = np.zeros((3, n_steps))
        self.trajectory[:, 0] = (self.position, self.speed, self.acceleration)
        self._step_index = 0

    def step(self, lead_position: float, lead_speed: float, dt: float):
        """Advance one step behind the host's leader, and record the new state."""
        super().step(lead_position, lead_speed, dt)
        self._step_index += 1
        self.trajectory[:, self._step_index] = (self.position, self.speed, self.acceleration)


def attach_ghosts(platoon: list[Vehicle], ghost_controllers, n_steps: int, controller_params=None):
    """
    Give every WTJ vehicle one ghost per controller name in ghost_controllers.
    controller_params ({name: {param: value}}) overrides controller defaults, as in sim().
    """
    unknown = [name for name in ghost_controllers if name not in CONTROLLERS]
    if unknown:
        raise ValueError(f"Unknown ghost controller(s) {unknown}; choose from {list(CONTROLLERS)}")
    controller_params = controller_params or {}
    for vehicle in platoon:
        if vehicle.type == HOST_TYPE:
            vehicle.ghosts = [GhostCar(vehicle, make_controller(name, controller_params.get(name)), n_steps)
                              for name in ghost_controllers]


@dataclass
class GhostResult:
    """
    WTJ host vs one of its ghosts. Work is total positive work,
    sum_t max(a * v, 0) * dt, in m^2/s^2 (no mass term).
    """
    car: int
    ghost_type: str
    host_total_work: float
    ghost_total_work: float
    ghost_min_gap: float  # Ghost's closest approach to the real car ahead (m); < 0 means it drove through it
    work_diff: float = field(init=False)  # ghost - host; positive means the ghost would have done more work

    def __post_init__(self):
        self.work_diff = self.ghost_total_work - self.host_total_work


def compare_ghosts(data: np.ndarray, platoon: list[Vehicle], dt: float) -> list[GhostResult]:
    """Compare every ghost in the platoon with its host, using the sim output `data`."""
    results = []
    for vehicle in platoon:
        host = data[vehicle.id]        # row k is car k; row 0 is the leader
        ahead = data[vehicle.id - 1]   # the real car both host and ghosts follow
        host_work = total_positive_work(host[1], host[2], dt)
        for ghost in vehicle.ghosts:
            results.append(GhostResult(
                car=vehicle.id,
                ghost_type=ghost.type,
                host_total_work=host_work,
                ghost_total_work=total_positive_work(ghost.trajectory[1], ghost.trajectory[2], dt),
                ghost_min_gap=float(np.min(ahead[0] - ghost.trajectory[0])),
            ))
    return results
