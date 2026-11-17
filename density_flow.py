"""
Density and flow of a run, measured over boxes in the time-space plane.

The time-space plane is cut into boxes of size dx (m) by dT (s). For each box,
with d_i the distance vehicle i travels inside it and TT_i the time it spends
inside it (lecture 1, "Given a time space diagram, how do you compute flow,
density and speed?"):

    flow     q   = sum(d_i)  / (dx * dT)
    density  rho = sum(TT_i) / (dx * dT)
    speed    v   = q / rho = sum(d_i) / sum(TT_i)

Only real vehicles count (leader included, ghosts left out). The platoon is a
moving block of cars on an empty road, so a box is only used if it lies
entirely between the last car and the leader for its whole time interval;
boxes that are partly empty road would read as falsely low density.

Run it to save each run's box values to <run folder>/density_flow_dx<dx>_dT<dT>.csv:

    uv run density_flow.py                  # most recent run
    uv run density_flow.py RUN [RUN ...] --dx 500 --dT 60
    uv run density_flow.py --all
"""

import argparse
import csv

import numpy as np

from run_io import add_run_arguments, load_run, resolve_runs

DEFAULT_DX = 250.0  # m
DEFAULT_DT = 30.0   # s

COLUMNS = ["t0_s", "x0_m", "density_veh_per_km", "flow_veh_per_hr", "speed_mps"]


def compute_density_and_flow(run, dx=DEFAULT_DX, dT=DEFAULT_DT):
    """
    Density, flow and speed for every dx-by-dT box fully inside the platoon.

    Returns a dict of 1D arrays, one entry per box: t0, x0 (box lower corner,
    s and m), density (veh/km), flow (veh/hr), speed (m/s); plus the box size
    used as dx and dT. dT is rounded to a whole number of sim steps.
    """
    rows = run.rows(ghost=False)
    pos, vel, dt = run.position[rows], run.velocity[rows], run.dt
    steps_per_box = max(1, round(dT / dt))
    dT = steps_per_box * dt
    n_tb = pos.shape[1] // steps_per_box         # whole time boxes only
    pos, vel = pos[:, :n_tb * steps_per_box], vel[:, :n_tb * steps_per_box]

    # Box index of every (vehicle, step) sample
    x_origin = np.floor(pos.min() / dx) * dx
    n_xb = int(np.ceil((pos.max() - x_origin) / dx)) + 1
    ix = np.clip(((pos - x_origin) // dx).astype(int), 0, n_xb - 1)
    it = np.broadcast_to(np.arange(pos.shape[1]) // steps_per_box, pos.shape)
    box = (it * n_xb + ix).ravel()

    total_time = np.bincount(box, minlength=n_tb * n_xb) * dt                          # sum TT_i, veh*s
    total_dist = np.bincount(box, weights=(vel * dt).ravel(), minlength=n_tb * n_xb)   # sum d_i, veh*m

    # Keep box (j, i) only if the last car is behind its lower edge and the
    # leader is ahead of its upper edge for every step of time box j
    back = pos.min(axis=0).reshape(n_tb, steps_per_box).max(axis=1)
    front = pos.max(axis=0).reshape(n_tb, steps_per_box).min(axis=1)
    x_lo = x_origin + np.arange(n_xb) * dx
    inside = (back[:, None] <= x_lo[None, :]) & (front[:, None] >= x_lo[None, :] + dx)
    j, i = np.nonzero(inside)
    keep = j * n_xb + i

    area = dx * dT
    flow = total_dist[keep] / area               # veh/s
    density = total_time[keep] / area            # veh/m
    return {
        "t0": j * dT, "x0": x_lo[i],
        "density": density * 1000, "flow": flow * 3600,
        "speed": np.divide(flow, density, out=np.zeros_like(flow), where=density > 0),
        "dx": dx, "dT": dT,
    }


def csv_path(run_dir, dx, dT):
    return run_dir / f"density_flow_dx{dx:g}_dT{dT:g}.csv"


def save_density_and_flow(run_dir, result):
    path = csv_path(run_dir, result["dx"], result["dT"])
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(zip(result["t0"].round(2), result["x0"].round(2), result["density"].round(3),
                             result["flow"].round(2), result["speed"].round(3)))
    return path


def density_and_flow_for(run_dir, dx=DEFAULT_DX, dT=DEFAULT_DT):
    """
    Box values for one run folder: read from its saved CSV when there is one
    for this box size, otherwise computed from timeseries.csv and saved.
    """
    run = None
    dT_used = dT
    path = csv_path(run_dir, dx, dT)
    if not path.is_file():
        # dT snaps to whole sim steps, so the saved name can differ slightly from the request
        run = load_run(run_dir)
        dT_used = max(1, round(dT / run.dt)) * run.dt
        path = csv_path(run_dir, dx, dT_used)
    if path.is_file():
        values = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
        return {"t0": values[:, 0], "x0": values[:, 1], "density": values[:, 2],
                "flow": values[:, 3], "speed": values[:, 4], "dx": dx, "dT": dT_used}
    result = compute_density_and_flow(run, dx, dT)
    save_density_and_flow(run_dir, result)
    return result


def main():
    parser = argparse.ArgumentParser(description="Compute density and flow over boxes for runs.")
    add_run_arguments(parser, show=False)
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    args = parser.parse_args()

    for run_dir in resolve_runs(args.runs, all_runs=args.all):
        result = compute_density_and_flow(load_run(run_dir), args.dx, args.dT)
        if len(result["flow"]) == 0:
            print(f"[{run_dir.name}] no {args.dx:g} m x {args.dT:g} s box fits inside the platoon")
            continue
        print(f"[{run_dir.name}] {len(result['flow'])} boxes, density "
              f"{result['density'].min():.1f}-{result['density'].max():.1f} veh/km, flow "
              f"{result['flow'].min():.0f}-{result['flow'].max():.0f} veh/hr -> "
              f"{save_density_and_flow(run_dir, result)}")


if __name__ == "__main__":
    main()
