import random
import numpy as np

from controllers import Controller, TimeGapController, WTJController

class Vehicle:
    def __init__(self, vehicle_id: int, vehicle_type: str, controller: Controller, mass: float, position: float = 0, speed: float = 0):
        self.id = vehicle_id
        self.type = vehicle_type
        self.controller = controller
        self.mass = mass
        self.position = position  # Initial position
        self.speed = speed        # Initial speed
        self.acceleration = 0     # Initial acceleration

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

def dynamics(cmd_accel, accel):
    """
    CarComplexSimulink -- discretized (also I think wrong...)
    """
    if cmd_accel >= 0:
        accel = .8395 * accel + .1605 * cmd_accel
    else:
        accel = .9277 * accel + .0723 * cmd_accel

    return accel

def sim(n_vehicles: int, penetration_rate: float, sim_length_seconds: float, dt: float = .05):
    """
    Simulate the traffic flow

    Parameters:
    n_vehicles: The total number of vehicles in the simulation.
    penetration_rate: The proportion of vehicles that run our controller (between 0 and 1).
    sim_length_seconds: The total length of the simulation in seconds.
    dt: The time step for the simulation.

    Returns:
    sim_output (np.ndarray): A 3D array containing the position, speed, and acceleration of each vehicle at each time step.
    time (np.ndarray): An array of time points corresponding to the simulation steps.
    platoon (list): A list of Vehicle objects representing the vehicles in the simulation.
    """
    n_steps = int(sim_length_seconds / dt)
    time = np.arange(0, sim_length_seconds, dt)

    # ================================================
    # Replace with lead data from bagfile
    lead_vehicle_vel = np.sin(np.linspace(0, 6 * np.pi, n_steps)) * 5 + 20
    lead_vehicle_pos = np.cumsum(lead_vehicle_vel * dt)
    lead_vehicle_accel = np.gradient(lead_vehicle_vel, dt)
    # ================================================
    
    platoon = []
    sim_output = np.zeros((n_vehicles+1, 3, n_steps))
    sim_output[0, 0, :] = lead_vehicle_pos
    sim_output[0, 1, :] = lead_vehicle_vel
    sim_output[0, 2, :] = lead_vehicle_accel

    for i in range(1, n_vehicles+1):
        random_value = random.random()
        vehicle = Vehicle(
            vehicle_id=i,
            vehicle_type='WTJ' if penetration_rate < random_value else 'TG',
            controller=WTJController() if penetration_rate < random_value else TimeGapController(),
            mass=sample_weight(),
            position=-50 * i,
            speed=20
        )
        platoon.append(vehicle)
        sim_output[i, 0, 0] = vehicle.position
        sim_output[i, 1, 0] = vehicle.speed
        sim_output[i, 2, 0] = 0

    for step in range(1, n_steps):
        for i in range(len(platoon)):
            if i == 0:
                lead_speed = lead_vehicle_vel[step]
                lead_position = lead_vehicle_pos[step]
            else:
                lead_speed = platoon[i - 1].speed
                lead_position = platoon[i - 1].position
            ego = platoon[i]
            cmd_accel = ego.controller.cmd_accel(sn=lead_position-ego.position, vn=ego.speed, sndot=lead_speed-ego.speed)
            ego.acceleration = dynamics(cmd_accel, ego.acceleration)
            ego.speed += ego.acceleration * dt
            ego.position += ego.speed * dt
            sim_output[i+1, 0, step] = ego.position
            sim_output[i+1, 1, step] = ego.speed
            sim_output[i+1, 2, step] = ego.acceleration
    return sim_output, time, platoon