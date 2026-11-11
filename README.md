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
| `--bag NAME` | | A bag in `bags/`, or any path to a `.bag` |
| `--all-bags` | | Run once per `.bag` in `bags/` |
| `--start S` | 0 | Seconds into the bag to start the lead profile |
| `--end S` | bag end | Seconds into the bag to stop |
| `--bag-topic T` | auto | Speed topic to use as the lead |
| `--n-vehicles N` | 100 | Number of followers |
| `--penetration-rate P` | 0.2 | Fraction of followers running WTJ (0 to 1); the rest run TG |
| `--dt DT` | 0.05 | Time step in seconds |
| `--wtj KEY=VALUE ...` | | Override WTJ controller parameters, e.g. `--wtj c=0.2 tau_min=1.5`. See [controller parameters](#controller-parameters) |
| `--tg KEY=VALUE ...` | | Override TG controller parameters, e.g. `--tg tau=2.0` |
| `--ghosts [CTRL ...]` | off | Give every WTJ car one ghost per listed controller (`TG`, `WTJ`); with no list, one `TG` ghost. See [Ghost cars](#ghost-cars) |
| `--no-plot` | | Skip saving plots, and record metrics only |
| `--show` | | Also open the plots in a window (single runs only) |

Without `--bag` or `--all-bags`, the leader follows a synthetic sine-wave speed
profile for `--length` seconds (default 360).

### Examples

```sh
# 5 minutes from the middle of a long drive, 50 followers, half running WTJ
uv run main.py --bag realRun --start 300 --end 600 --n-vehicles 50 --penetration-rate 0.5

# Sweep penetration rates over every bag, metrics only
for p in 0 0.25 0.5 0.75 1; do uv run main.py --all-bags --penetration-rate $p --no-plot; done
```

Controller assignment and vehicle masses are random, so repeated runs with the
same settings give slightly different results.

## Controller parameters

Every parameter of the WTJ and TG controllers in `controllers.py` can be changed
from the command line with `KEY=VALUE` pairs:

| Controller | Parameters (defaults) |
|---|---|
| `--wtj` | `alpha` (1.0), `tau_min` (2.0), `tau_max` (3.0), `gmin` (15), `lam` (0.5), `c` (0.1) |
| `--tg` | `alpha` (1.0), `tau` (2.5), `gmin` (15), `lam` (0.5) |

```sh
uv run main.py --bag realRun --wtj c=0.2
uv run main.py --bag realRun --wtj c=0.2 tau_min=1.5 --tg tau=2.0 --ghosts

# Sweep c (bash)
for c in 0.05 0.1 0.2 0.4; do uv run main.py --bag realRun --wtj c=$c --ghosts --no-plot; done
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
name, number of followers, penetration rate).

**Plots** go to `plots/<graph>/<run>.png`, one folder per graph:

- `positions/`, `speeds/`, `accelerations/`: one line per vehicle. The lead is
  green, WTJ cars are blue, TG cars are red.
- `positive_work/`: one bar per follower, with the run average as a dashed
  line. With `--ghosts`, faint ghost bars are drawn behind the WTJ cars' bars.

**Metrics** are added to two CSVs in `results/`. Each run adds rows to them, so
results from many runs collect in one place:

- `runs.csv`: one row per run with its settings, WTJ/TG counts and the average
  positive work across followers.
- `cars.csv`: one row per follower with its type, mass and positive work.

Average positive work for each follower is

$$\frac{1}{T}\sum_t \max\big(a(t)\,v(t),\ 0\big)\,\Delta t$$

where $T$ is the sim length. It has no mass term, so the units are m²/s³
(W/kg). The leader is not included.

## Using it from Python or the notebook

```python
from lead import load_lead_from_bag
from sim import sim
from metrics import positive_work_per_car

lead_vel = load_lead_from_bag("bags/realRun.bag", dt=0.05, start=300, end=600)
data, time, platoon = sim(n_vehicles=50, penetration_rate=0.5, dt=0.05, lead_vel=lead_vel)
work = positive_work_per_car(data, dt=0.05)
```

`data` has shape `(n_vehicles + 1, 3, n_steps)`. Row 0 is the leader, and the
second axis is position, speed and acceleration. To use `notebook.ipynb`, select
the `.venv` kernel in VS Code.

## Files

| File | Contents |
|---|---|
| `main.py` | Command-line entry point: runs sims, saves plots and CSVs |
| `sim.py` | Platoon simulation |
| `vehicles.py` | `Vehicle`, actuator dynamics, vehicle mass sampling |
| `ghosts.py` | `GhostCar`, `attach_ghosts`, `compare_ghosts` |
| `controllers.py` | `TimeGapController`, `WTJController` (`IDMController` is a stub) |
| `lead.py` | Loads a lead speed profile from a bag |
| `metrics.py` | Average positive work per car |
| `notebook.ipynb` | Interactive exploration |
