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

TAU_ACCEL_REF = 1 / 3.5   # s, throttle/powertrain lag for the reference car
TAU_BRAKE_REF = 1 / 1.5   # s, brake lag for the reference car

def dynamics(cmd_accel, accel, mass=None, reference_mass=1500.0, dt=0.05,
             alpha=0.5, variable_mass=False, accel_limits=(-3.0, 1.5)):
    """
    First-order acceleration lag, a/(s+a), discretized exactly (ZOH).

    Physical model: actuator force changes at a rate proportional to the
    acceleration error, dF/dt = k (a_cmd - a), with F = m a. That gives
    tau = m / k. With actuator authority scaling as k ~ m^(1 - alpha),
        tau(m) = tau_ref * (m / m_ref)^alpha
    alpha=0: mass has no effect; alpha=1: same actuator, heavier body.

    accel_limits: optional (a_min, a_max) in m/s^2 for this vehicle
    (e.g. from power/mass and braking capability). This is saturation,
    not a gain change.
    """
    if accel_limits is not None:
        cmd_accel = np.clip(cmd_accel, *accel_limits)

    mass_ratio = 1.0
    if variable_mass:
        if mass is None:
            raise ValueError("mass is required when variable_mass=True")
        mass_ratio = (mass / reference_mass) ** alpha

    tau = (TAU_ACCEL_REF if cmd_accel >= 0 else TAU_BRAKE_REF) * mass_ratio
    phi = np.exp(-dt / tau)  # exact ZOH pole; equals .8395 / .9277 at m_ref

    return phi * accel + (1 - phi) * cmd_accel