import numpy as np

from controllers import Controller


class Vehicle:
    def __init__(self, vehicle_id: int, controller: Controller, mass: float, position: float = 0,
                 speed: float = 0, variable_mass: bool = False):
        self.id = vehicle_id
        self.controller = controller
        self.mass = mass
        self.variable_mass = variable_mass
        self.position = position  # Initial position
        self.speed = speed        # Initial speed
        self.acceleration = 0     # Initial acceleration
        self.ghosts = []          # GhostCars riding along with this vehicle (WTJ only; see ghosts.py)

    @property
    def type(self) -> str:
        """Controller name, e.g. "WTJ" or "TG"."""
        return self.controller.name

    def step(self, lead_position: float, lead_speed: float, dt: float):
        """Advance one time step following a vehicle at lead_position / lead_speed."""
        cmd_accel = self.controller.cmd_accel(sn=lead_position - self.position, vn=self.speed,
                                              sndot=lead_speed - self.speed)
        self.acceleration = dynamics(cmd_accel, self.acceleration, self.mass,
                                     variable_mass=self.variable_mass)
        self.speed += self.acceleration * dt
        self.position += self.speed * dt


def sample_weight():
    """
    Generates a single vehicle weight (in kg) from a bimodal lognormal
    distribution representing mixed traffic (85% cars, 15% trucks).
    """
    # Decide vehicle type: 0 for Passenger Car, 1 for Heavy Truck
    vehicle_type = np.random.choice([0, 1], p=[0.85, 0.15])

    # Sample from the corresponding lognormal distribution
    if vehicle_type == 0:
        # Car parameters (~1,500 kg average)
        mu = np.log(1500)
        sigma = 0.25
    else:
        # Truck parameters (~15,000 kg average)
        mu = np.log(15000)
        sigma = 0.40

    # Generate lognormal sample: e^(mu + sigma * Z)
    weight = np.exp(np.random.normal(mu, sigma))

    return weight


def dynamics(cmd_accel, accel, mass=None, reference_mass=1500, variable_mass=False):
    """
    CarComplexSimulink -- discretized (also I think wrong...)

    With variable_mass enabled, scale commanded acceleration by mass so heavier
    vehicles respond more slowly than the reference passenger car.
    """
    effective_cmd_accel = cmd_accel
    if variable_mass:
        if mass is None:
            raise ValueError("mass is required when variable_mass=True")
        mass_factor = reference_mass / mass
        mass_factor = np.clip(mass_factor, 0.2, 1.5)
        effective_cmd_accel = cmd_accel * mass_factor

    if effective_cmd_accel >= 0:
        accel = .8395 * accel + .1605 * effective_cmd_accel
    else:
        accel = .9277 * accel + .0723 * effective_cmd_accel

    return accel
