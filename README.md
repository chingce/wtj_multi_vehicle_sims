# wtj_multi_vehicle_sims

Python simulations of a platoon of vehicles following a lead vehicle. Each
follower runs either the WTJ controller or a plain time-gap (TG) controller, and
the lead vehicle's speed can be taken from a recorded ROS1 bag file.

## Setup

The project uses [uv](https://docs.astral.sh/uv/) and Python 3.14 (pinned in
`.python-version`). From the repo root:

```sh
uv sync
```

This creates `.venv` with numpy, matplotlib and rosbags. You don't need a ROS
install to read bags.

## Running a sim with a bag file as the leader

1. Put your `.bag` files in the `bags/` folder. Bag files are git-ignored
   because of their size, so each person adds their own.
2. Run one bag by name, with or without `.bag`:

   ```sh
   uv run main.py --bag realRun
   ```

   Or run every bag in `bags/`:

   ```sh
   uv run main.py --all-bags
   ```

The sim lasts as long as the bag, or as long as the window you choose with
`--start`/`--end`. Followers start 50 m apart at the lead's initial speed.

### Which topic is used as the lead speed

The leader's speed is read from the first of these topics that the bag
contains, then resampled to the sim's time step:

| Topic | Used for |
|---|---|
| `/leadcar/car/state/vel_x` | Multi-car bags: the actual lead car's speed |
| `/car/state/vel_x` | Single-car bags: the recording car's own drive becomes the lead trajectory |

To read a different topic, pass `--bag-topic`. It must be a `std_msgs/Float64`
speed in m/s.

### Options

| Flag | Default | Meaning |
|---|---|---|
| `--bag NAME [NAME ...]` | | One or more bags in `bags/` (with or without `.bag`), or paths. Several bags x several rates all run in parallel |
| `--all-bags` | | Run once per `.bag` in `bags/` |
| `--start S` | 0 | Seconds into the bag to start the lead profile |
| `--end S` | bag end | Seconds into the bag to stop |
| `--bag-topic T` | auto | Speed topic to use as the lead |
| `--n-vehicles N` | 100 | Number of followers |
| `--penetration-rate P [P ...]` | 0.2 | Fraction of followers running the `--smart` controller (0 to 1); the rest run the `--base` controller. Give several values to run a sweep in parallel, e.g. `--penetration-rate 0.25 0.5 0.75 1` |
| `--jobs N` | all, up to CPU count | How many simulations (rates x bags) run at once. Each runs in its own process and writes its own run folder; only the main process writes `runs.csv`, `cars.csv` and `ghosts.csv`. `--jobs 1` runs them one at a time. Results are identical either way |
| `--smart CTRL` | `WTJ` | Our controller: `WTJ` (the original, validated against MATLAB), `WTJ2` (WTJ with `lam` 2), `WTJ3` (WTJ without the hard lower bound `al`: targets `min(beta*al + (1-beta)*a_star, au)`, with `lam` 3 and `beta` 0.25; see `--wtj3`), or `WTJ4` (WTJ3 with `beta` 0.1). `TG` is also allowed, for comparison. These cars get the ghosts. Non-default choices are added to the run name (e.g. `_WTJ3`) |
| `--base CTRL` | `IDM` | Controller for every other follower: `IDM` or `TG`. Added to the run name (`..._p0.5_IDM`) |
| `--seed N` | random | Seed for the controller draw and masses. Runs with the same seed put the same kind of car, with the same mass, in each slot, so `--smart WTJ3` vs `--smart WTJ4` is a like-for-like comparison |
| `--allow-collisions` | off | Keep simulating when a car reaches the car ahead. By default the run stops at the first collision, writes only a `runs.csv` row with the time and cars in its `collision` column, and moves on |
| `--dt DT` | 0.05 | Time step in seconds |
| `--variable-mass` | off | Scale each vehicle's acceleration response by its sampled mass; outputs go to `variable_mass_results/` |
| `--wtj KEY=VALUE ...` | | Override WTJ controller parameters, e.g. `--wtj c=0.2 tau_min=1.5`. See [controller parameters](#controller-parameters) |
| `--tg KEY=VALUE ...` | | Override TG controller parameters, e.g. `--tg tau=2.0` |
| `--ghosts [CTRL ...]` | off | Give every WTJ car one ghost per listed controller (`TG`, `WTJ`); with no list, one `TG` ghost. See [Ghost cars](#ghost-cars) |

Without `--bag` or `--all-bags`, the leader follows a synthetic sine-wave speed
profile for `--length` seconds (default 360).

### Examples

```sh
# 5 minutes from the middle of a long drive, 50 followers, half running WTJ
uv run main.py --bag realRun --start 300 --end 600 --n-vehicles 50 --penetration-rate 0.5

# Sweep penetration rates over every bag (all runs in parallel)
uv run main.py --all-bags --penetration-rate 0 0.25 0.5 0.75 1
```

`main.py` only writes CSVs. To make plots, run the [plotting scripts](#plotting)
on the run afterwards.

Controller assignment and vehicle masses are random, so repeated runs with the
same settings give slightly different results. By default, sampled mass is
recorded but does not affect dynamics. With `--variable-mass`, mass scales the
actuator dynamics: heavier vehicles respond more slowly than the 1,500 kg
reference car.

## Controller parameters

Every parameter of the WTJ and TG controllers in `controllers.py` can be changed
from the command line with `KEY=VALUE` pairs:

| Controller | Parameters (defaults) |
|---|---|
| `--wtj` | `alpha` (1.0), `tau_min` (2.0), `tau_max` (3.0), `gmin` (15), `lam` (0.5), `c` (0.1) |
| `--wtj2` | Same as `--wtj`, but `lam` defaults to 2.0 |
| `--wtj3` | Same as `--wtj` but `lam` defaults to 3.0, plus `beta` (0.25): target acceleration is `beta*al + (1-beta)*a_star`, capped by `au` |
| `--wtj4` | Same as `--wtj3`, but `beta` defaults to 0.1 |
| `--tg` | `alpha` (1.0), `tau` (2.5), `gmin` (15), `lam` (0.5) |

```sh
uv run main.py --bag realRun --wtj c=0.2
uv run main.py --bag realRun --wtj c=0.2 tau_min=1.5 --tg tau=2.0 --ghosts

# Sweep c (bash)
for c in 0.05 0.1 0.2 0.4; do uv run main.py --bag realRun --wtj c=$c --ghosts; done
```

The overrides apply to ghosts of that controller too. Changed parameters are
added to the run name (e.g. `..._p0.5_WTJ-c0.2`), the plot titles, and the
`controller_params` column of `runs.csv` (`defaults` when nothing was
changed). From Python, pass `controller_params={"WTJ": {"c": 0.2}}` to `sim()`.

## Ghost cars

A ghost answers the question: what if this WTJ car had run a different
controller, with the rest of traffic unchanged?

When ghosts are on, **every WTJ car** gets one ghost per controller you ask
for. A ghost starts with its WTJ car's position and speed, then at every step
follows the same real car the WTJ car follows, using its own controller and the
same actuator lag. Nothing follows a ghost, so ghosts never change the rest of
the platoon. TG cars never get ghosts.

```sh
uv run main.py --bag realRun                     # no ghosts (default)
uv run main.py --bag realRun --ghosts            # one TG ghost per WTJ car
uv run main.py --bag realRun --ghosts TG WTJ     # a TG ghost and a WTJ ghost per WTJ car
```

The ghost controllers are the names in `CONTROLLERS` in `controllers.py`. A WTJ
ghost on a WTJ car copies its host exactly, which is a useful sanity check.

Each ghost adds a row to `results/ghosts.csv`. The row has the WTJ car's and
the ghost's total positive work, Σ max(a·v, 0)·Δt in m²/s², and `work_diff` =
ghost − WTJ car. A positive `work_diff` means the ghost's controller would have
done more work in that position. The row also records `ghost_min_gap`, the
ghost's closest approach to the real car ahead.

In the plot `plots/positive_work/<run>.png`, each ghost appears as a faint
bar (20% opacity) behind its WTJ car's bar (60% opacity), colored by the
ghost's controller. Ghost bars use the same average-rate units as the real
bars (ghost total work / sim length), so where a ghost bar sticks out above
its WTJ bar, the ghost's controller would have done more work.

From Python, pass `ghosts` to `sim()`. Without it, `sim()` runs exactly as
before, with no ghosts:

```python
from sim import sim
from ghosts import compare_ghosts

data, time, platoon = sim(100, 0.2, lead_vel=lead_vel, ghosts=["TG"])  # or ghosts=True for the default
for r in compare_ghosts(data, platoon, dt=0.05):
    print(r.car, r.ghost_type, r.work_diff)

platoon[4].ghosts                 # car 5's ghosts ([] if it is not WTJ)
platoon[4].ghosts[0].trajectory   # shape (3, n_steps): position, speed, acceleration
```

## Output

Each run is named like `20261110-192252_realRun_n50_p0.5` (timestamp, bag
name, number of followers, penetration rate). Outputs go to `results/`, or to
`variable_mass_results/results/` with `--variable-mass`. `main.py` makes no
plots; see [Plotting](#plotting).

**Per-run data** goes to `results/runs/<run>/` as two CSVs:

- `vehicles.csv`: one row per vehicle: `vehicle` (0 is the leader, then 1, 2, …
  behind it), `type` (`lead`, `WTJ`, `TG`, `IDM`), `ghost` (`True`/`False`) and
  `mass_kg`. A ghost has the same number and mass as its WTJ car, and `type` is
  the ghost's own controller.
- `timeseries.csv`: one row per vehicle per time step: `vehicle`, `type`,
  `ghost`, `time_s`, `position_m`, `velocity_mps`, `acceleration_mps2`.
  `(vehicle, type, ghost)` identifies one trajectory.

`timeseries.csv` is large: about 2.3M rows (~100 MB) for 100 cars over the
19-minute `realRun` bag, which is at GitHub's 100 MB file limit.

**Metrics** are added to two CSVs in `results/`. Each run adds rows to them, so
results from many runs collect in one place:

- `runs.csv`: one row per run with its settings, WTJ/TG counts, whether
  variable mass was enabled, and the average positive work across followers.
- `cars.csv`: one row per follower with its type, mass, whether variable mass
  was enabled, and positive work.

Average positive work for each follower is

$$\frac{1}{T}\sum_t \max\big(a(t)\,v(t),\ 0\big)\,\Delta t$$

where $T$ is the sim length. It has no mass term, so the units are m²/s³
(W/kg). With `--variable-mass`, mass affects the simulated vehicle dynamics but
is still not multiplied into this metric. The leader is not included.

## Plotting

The plotting scripts read a run's CSVs, so you can re-plot without re-running
the sim. Each takes run names (or run folders); with none, it uses the most
recent run, and `--all` uses every run. `--show` also opens the plots.

```sh
uv run plot_run.py                                  # positions, speeds, accelerations, positive work
uv run plot_time_space.py                           # time-space diagram of the most recent run
uv run plot_fd.py --all --match realRun             # one fundamental diagram, every realRun run
```

| Script | Plot | Shows |
|---|---|---|
| `plot_run.py` | `plots/positions/`, `speeds/`, `accelerations/` | One line per vehicle: lead green, WTJ blue, TG red, IDM orange. Ghosts left out |
| `plot_run.py` | `plots/positive_work/` | One bar per follower, run average dashed; ghosts as faint bars behind their WTJ car |
| `plot_time_space.py` | `plots/time_space/` | Trajectories colored by speed, with the density/flow boxes outlined |
| `plot_fd.py` | `plots/fundamental_diagram/` | Flow vs density for all selected runs on one plot, each point colored by its run's penetration rate |

Per-run plots go under `variable_mass_results/plots/` for variable-mass runs.

### Density and flow

`density_flow.py` measures density and flow over boxes in the time-space plane
(`--dx` m × `--dT` s, default 250 m × 30 s). For each box, with $d_i$ the
distance vehicle $i$ travels inside it and $TT_i$ the time it spends there:

$$q = \frac{\sum_i d_i}{\Delta x\,\Delta T}, \qquad \rho = \frac{\sum_i TT_i}{\Delta x\,\Delta T}, \qquad v = \frac{q}{\rho}$$

Only real vehicles count (ghosts are left out). The platoon is a block of cars
on an otherwise empty road, so a box is only used if it lies between the last
car and the leader for its whole time interval. Boxes that are partly empty
road would read as falsely low density.

The box values are saved to `results/runs/<run>/density_flow_dx<dx>_dT<dT>.csv`
(density in veh/km, flow in veh/hr, speed in m/s). `plot_fd.py` and
`plot_time_space.py` reuse that file when it exists for the box size asked
for, and compute and save it when it doesn't. To compute it on its own:

```sh
uv run density_flow.py --all --dx 500 --dT 60
```

### Fundamental diagram across penetration rates

`plot_fd.py` puts every selected run on one flow-density plot. It reads each
run's penetration rate from its name (`..._n100_p0.25`) and colors the points
on a light-to-dark blue scale, from low to high penetration. Runs with the same
rate share a color.

Use `--match` so one diagram doesn't mix different bags or controller settings:

```sh
for p in 0 0.25 0.5 0.75 1; do uv run main.py --bag realRun --penetration-rate $p; done
uv run plot_fd.py --all --match realRun_n100          # -> plots/fundamental_diagram/realRun_n100_5runs_dx250_dT30.png
uv run plot_fd.py RUN_A RUN_B --name my_comparison    # chosen runs, chosen file name
```

From Python, `run_io.load_run(folder)` returns a run as arrays (`time`, and
`position`, `velocity`, `acceleration` with one row per trajectory in
`vehicles` order), and `density_flow.compute_density_and_flow(run, dx, dT)`
returns the box values.

## Using it from Python or the notebook

```python
from lead import load_lead_from_bag
from sim import sim
from metrics import positive_work_per_car

lead_vel = load_lead_from_bag("bags/realRun.bag", dt=0.05, start=300, end=600)
data, time, platoon = sim(n_vehicles=50, penetration_rate=0.5, dt=0.05, lead_vel=lead_vel)
# or add variable_mass=True to scale acceleration response by sampled mass
work = positive_work_per_car(data, dt=0.05)
```

`data` has shape `(n_vehicles + 1, 3, n_steps)`. Row 0 is the leader, and the
second axis is position, speed and acceleration. To use `notebook.ipynb`, select
the `.venv` kernel in VS Code.

## Files

| File | Contents |
|---|---|
| `main.py` | Command-line entry point: runs sims and saves CSVs |
| `run_io.py` | Writes and loads the per-run `vehicles.csv` / `timeseries.csv` |
| `plot_run.py` | Positions, speeds, accelerations and positive work plots from a run's CSVs |
| `density_flow.py` | Density and flow over time-space boxes; saves them per run |
| `plot_time_space.py` | Time-space diagram of a run |
| `plot_fd.py` | One fundamental diagram across runs, colored by penetration rate |
| `sim.py` | Platoon simulation |
| `vehicles.py` | `Vehicle`, optional mass-scaled actuator dynamics, vehicle mass sampling |
| `ghosts.py` | `GhostCar`, `attach_ghosts`, `compare_ghosts` |
| `controllers.py` | `TimeGapController`, `WTJController`, `WTJ2Controller`, `WTJ3Controller`, `WTJ4Controller`, `IDMController` |
| `lead.py` | Loads a lead speed profile from a bag |
| `metrics.py` | Average positive work per car |
| `notebook.ipynb` | Interactive exploration |
