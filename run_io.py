"""
Per-run CSVs: writing them from a sim, and loading them back for plotting.

Each run gets a folder <results>/runs/<run_name>/ holding:
    vehicles.csv    one row per vehicle: vehicle, type, ghost, mass_kg
    timeseries.csv  one row per vehicle per time step:
                    vehicle, type, ghost, time_s, position_m, velocity_mps, acceleration_mps2

Vehicle 0 is the leader (type "lead"); followers are numbered 1, 2, ... behind it.
A ghost has its WTJ car's number and mass, `ghost` True, and its own controller as
`type`, so (vehicle, type, ghost) identifies one trajectory.
"""

import csv
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent
RUN_DIRS = (ROOT / "results" / "runs", ROOT / "variable_mass_results" / "results" / "runs")

TIMESERIES_HEADER = "vehicle,type,ghost,time_s,position_m,velocity_mps,acceleration_mps2"


def _trajectories(data, platoon):
    """(number, type, is_ghost, mass, (3, n_steps) array) for leader, followers and ghosts."""
    yield 0, "lead", False, None, data[0]
    for v in platoon:
        yield v.id, v.type, False, v.mass, data[v.id]
        for g in v.ghosts:
            yield v.id, g.type, True, g.mass, g.trajectory


def write_run_csvs(run_dir, data, time, platoon):
    """Write vehicles.csv and timeseries.csv for one sim into run_dir."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    trajectories = list(_trajectories(data, platoon))

    with open(run_dir / "vehicles.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["vehicle", "type", "ghost", "mass_kg"])
        for number, vtype, is_ghost, mass, _ in trajectories:
            writer.writerow([number, vtype, is_ghost, "" if mass is None else round(mass, 1)])

    with open(run_dir / "timeseries.csv", "w", newline="") as f:
        f.write(TIMESERIES_HEADER + "\n")
        for number, vtype, is_ghost, _, traj in trajectories:
            # Labels are constant per trajectory, so bake them into the format string
            np.savetxt(f, np.column_stack([time, traj.T]),
                       fmt=f"{number},{vtype},{is_ghost},%.2f,%.4f,%.4f,%.4f")


@dataclass
class Run:
    """
    One run loaded from its CSVs. Row k of every array is trajectory k, in the
    order of `vehicles` (the order the CSVs were written: leader, then each
    follower followed by its ghosts).
    """
    name: str
    dir: Path
    vehicles: list[dict]      # {"vehicle": int, "type": str, "ghost": bool, "mass_kg": float | None}
    time: np.ndarray          # (n_steps,)
    position: np.ndarray      # (n_trajectories, n_steps)
    velocity: np.ndarray
    acceleration: np.ndarray

    @property
    def dt(self) -> float:
        return float(self.time[1] - self.time[0])

    @property
    def variable_mass(self) -> bool:
        return "variable_mass_results" in self.dir.parts

    @property
    def plots_dir(self) -> Path:
        """plots/ next to the results/ folder this run lives in."""
        return self.dir.parents[2] / "plots"

    def collisions(self):
        """
        Real followers that reached the car ahead at some point (gap <= 0), as
        (vehicle number, first collision time in s). Empty for a clean run.
        """
        real = self.rows(ghost=False)       # leader, then followers in order
        gap = self.position[real][:-1] - self.position[real][1:]
        hit = np.nonzero((gap <= 0).any(axis=1))[0]
        return [(self.vehicles[real[i + 1]]["vehicle"], float(self.time[np.argmax(gap[i] <= 0)])) for i in hit]

    def collision_note(self):
        """'' for a clean run, else e.g. ' - 19 COLLISIONS (first at t=136.2 s)' for plot titles."""
        hits = self.collisions()
        if not hits:
            return ""
        return f" - {len(hits)} COLLISION{'S' * (len(hits) > 1)} (first at t={min(t for _, t in hits):.1f} s)"

    def rows(self, ghost=False):
        """Indices of the real vehicles (leader included), or of the ghosts with ghost=True."""
        return [k for k, v in enumerate(self.vehicles) if v["ghost"] == ghost]


def load_run(run_dir) -> Run:
    """Load a run folder written by write_run_csvs."""
    run_dir = Path(run_dir)
    with open(run_dir / "vehicles.csv", newline="") as f:
        vehicles = [{"vehicle": int(r["vehicle"]), "type": r["type"], "ghost": r["ghost"] == "True",
                     "mass_kg": float(r["mass_kg"]) if r["mass_kg"] else None}
                    for r in csv.DictReader(f)]

    # Numeric columns only (vehicle, time, position, velocity, acceleration); fast C parser
    values = np.loadtxt(run_dir / "timeseries.csv", delimiter=",", skiprows=1, usecols=(0, 3, 4, 5, 6))
    n_traj = len(vehicles)
    if len(values) % n_traj:
        raise ValueError(f"{run_dir}: {len(values)} rows don't split evenly into {n_traj} trajectories")
    values = values.reshape(n_traj, -1, 5)
    if not np.array_equal(values[:, 0, 0], [v["vehicle"] for v in vehicles]):
        raise ValueError(f"{run_dir}: timeseries.csv is not in the same order as vehicles.csv")

    return Run(name=run_dir.name, dir=run_dir, vehicles=vehicles, time=values[0, :, 1],
               position=values[:, :, 2], velocity=values[:, :, 3], acceleration=values[:, :, 4])


def resolve_runs(names, all_runs=False):
    """
    Turn command-line run arguments into run folders. Each name may be a path or
    a run name inside results/runs/ or variable_mass_results/results/runs/.
    With no names, returns the most recently written run.
    """
    found = sorted((d for base in RUN_DIRS if base.is_dir() for d in base.iterdir()
                    if (d / "timeseries.csv").is_file()), key=lambda d: d.stat().st_mtime)
    if all_runs:
        return found
    if not names:
        if not found:
            raise FileNotFoundError(f"No runs found in {', '.join(map(str, RUN_DIRS))}")
        return found[-1:]
    dirs = []
    for name in names:
        candidates = [Path(name)] + [base / name for base in RUN_DIRS]
        match = next((c for c in candidates if (c / "timeseries.csv").is_file()), None)
        if match is None:
            raise FileNotFoundError(f"No run {name!r} (looked for a folder with timeseries.csv)")
        dirs.append(match)
    return dirs


def penetration_rate(run_name):
    """The penetration rate a run was set to, from its name (e.g. '..._n100_p0.25' -> 0.25)."""
    match = re.search(r"_n\d+_p([0-9.]+)", run_name)
    if match is None:
        raise ValueError(f"Can't find a penetration rate (_n<N>_p<P>) in run name {run_name!r}")
    return float(match.group(1))


def add_run_arguments(parser, show=True):
    """Shared command-line arguments for picking runs."""
    parser.add_argument("runs", nargs="*",
                        help="run folders or run names (default: the most recent run)")
    parser.add_argument("--all", action="store_true", help="every run in results/runs/ folders")
    if show:
        parser.add_argument("--show", action="store_true", help="also open the plots in a window")
