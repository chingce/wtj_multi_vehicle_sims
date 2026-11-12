import random
import numpy as np

from controllers import make_controller
from ghosts import DEFAULT_GHOSTS, attach_ghosts
# Re-exported so `from sim import dynamics` etc. keep working
from vehicles import Vehicle, dynamics, sample_weight  # noqa: F401


def sim(n_vehicles: int, penetration_rate: float, sim_length_seconds: float = None, dt: float = .05,
        lead_vel: np.ndarray = None, ghosts=None, controller_params: dict = None,
        variable_mass: bool = False):
    """
    Simulate the traffic flow

    Parameters:
    n_vehicles: The total number of vehicles in the simulation.
    penetration_rate: The proportion of vehicles that run our controller (between 0 and 1).
    sim_length_seconds: The total length of the simulation in seconds. Ignored when lead_vel is given.
    dt: The time step for the simulation.
    lead_vel: Optional lead vehicle speed at each time step (m/s), e.g. from lead.load_lead_from_bag.
        Defaults to a synthetic sine wave.
    ghosts: Ghost controllers to attach to every WTJ vehicle, as names from controllers.CONTROLLERS
        (e.g. ["TG"] or ["TG", "WTJ"]). True means the default, one TG ghost. None (default) means
        no ghosts. Ghosts never affect traffic; compare them with ghosts.compare_ghosts.
    controller_params: Optional overrides of controller defaults, keyed by controller name,
        e.g. {"WTJ": {"c": 0.2}, "TG": {"tau": 2.0}}. Applies to real vehicles and ghosts alike.
    variable_mass: If True, each vehicle's sampled mass scales its acceleration response.

    Returns:
    sim_output (np.ndarray): A 3D array containing the position, speed, and acceleration of each vehicle at each time step.
    time (np.ndarray): An array of time points corresponding to the simulation steps.
    platoon (list): A list of Vehicle objects representing the vehicles in the simulation.
    """
    if lead_vel is None:
        n_steps = int(sim_length_seconds / dt)
        lead_vehicle_vel = np.sin(np.linspace(0, 6 * np.pi, n_steps)) * 5 + 20
    else:
        lead_vehicle_vel = np.asarray(lead_vel, dtype=float)
        n_steps = len(lead_vehicle_vel)
    time = np.arange(n_steps) * dt
    lead_vehicle_pos = np.cumsum(lead_vehicle_vel * dt)
    lead_vehicle_accel = np.gradient(lead_vehicle_vel, dt)

    controller_params = controller_params or {}
    platoon = []
    sim_output = np.zeros((n_vehicles+1, 3, n_steps))
    sim_output[0, 0, :] = lead_vehicle_pos
    sim_output[0, 1, :] = lead_vehicle_vel
    sim_output[0, 2, :] = lead_vehicle_accel

    for i in range(1, n_vehicles+1):
        controller_name = "WTJ" if random.random() < penetration_rate else "TG"
        vehicle = Vehicle(
            vehicle_id=i,
            controller=make_controller(controller_name, controller_params.get(controller_name)),
            mass=sample_weight(),
            position=-50 * i,
            speed=lead_vehicle_vel[0],
            variable_mass=variable_mass
        )
        platoon.append(vehicle)
        sim_output[i, 0, 0] = vehicle.position
        sim_output[i, 1, 0] = vehicle.speed
        sim_output[i, 2, 0] = 0

    if ghosts is True:
        ghosts = DEFAULT_GHOSTS
    elif isinstance(ghosts, str):
        ghosts = [ghosts]
    if ghosts:
        attach_ghosts(platoon, ghosts, n_steps, controller_params)

    for step in range(1, n_steps):
        for i in range(len(platoon)):
            if i == 0:
                lead_speed = lead_vehicle_vel[step]
                lead_position = lead_vehicle_pos[step]
            else:
                lead_speed = platoon[i - 1].speed
                lead_position = platoon[i - 1].position
            ego = platoon[i]
            ego.step(lead_position, lead_speed, dt)
            for ghost in ego.ghosts:
                ghost.step(lead_position, lead_speed, dt)  # same leader as the host; nothing follows a ghost
            sim_output[i+1, 0, step] = ego.position
            sim_output[i+1, 1, step] = ego.speed
            sim_output[i+1, 2, step] = ego.acceleration
    return sim_output, time, platoon
