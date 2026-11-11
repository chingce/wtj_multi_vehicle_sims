"""Energy-style metrics computed from sim output."""

import numpy as np


def total_positive_work(speed, accel, dt):
    """
    Total positive work, sum_t max(a(t) * v(t), 0) * dt, along the last axis.

    There is no mass term, so units are m^2/s^2 (J/kg). Works on one
    trajectory (1D arrays) or many at once (2D arrays, one row per car).
    """
    return np.sum(np.maximum(accel * speed, 0), axis=-1) * dt


def positive_work_per_car(data, dt):
    """
    Average positive work rate for each car following the leader.

    For car i this is sum_t max(a_i(t) * v_i(t), 0) * dt / T, where T is the sim
    length: the time-average of positive acceleration times speed (no mass
    term, so units are m^2/s^3, i.e. W/kg).

    Parameters:
    data: Sim output of shape (n_vehicles+1, 3, n_steps); row 0 is the leader.
    dt: Sim time step in seconds.

    Returns:
    work (np.ndarray): One value per follower, in platoon order.
    """
    total_time = data.shape[2] * dt
    return total_positive_work(data[1:, 1, :], data[1:, 2, :], dt) / total_time
