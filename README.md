# wtj_multi_vehicle_sims

Simulations of a 100-vehicle platoon behind a recorded lead-vehicle drive, used
to develop and evaluate a smoothing car-following controller for mixed traffic.
The repository reproduces the report in `report/`: *From WTJ to the Special Time
Gap Controller*. It covers the design iteration, from the original WTJ controller
to the special time gap controller (STG), and the evaluation of the final
controller against a plain time gap controller (TG).

## Setup

The project uses [uv](https://docs.astral.sh/uv/) and Python 3.14 (pinned in
`.python-version`):

```sh
uv sync
```

This installs numpy, matplotlib and rosbags; reading bags needs no ROS install.
Put the four lead-vehicle bags in `bags/` (they are git-ignored because of their
size):

| Bag | Report label | Throughput marker |
|---|---|---|
| `wtj_real_drive.bag` | WTJ real drive | 2,000 m |
| `2025_10_29_20_50_04_2T3MWRFVXLW056972profacc_test.bag` | Stop-and-go test (2025-10-29) | 1,000 m |
| `2026_10_29_14_31_02_2T3MWRFVXLW056972profaccradar.bag` | Stop-and-go (2026-10-29) | 1,000 m |
| `2026_10_23_16_53_49_2T3MWRFVXLW056972profaccradar.bag` | Fast mixed (2026-10-23) | 10,000 m |

## Producing the report

```sh
uv run iteration.py   --results-dir C:/Users/me/wtj_results   # stages 1-6: runs + iteration tables
uv run final_model.py --results-dir C:/Users/me/wtj_results   # STG vs TG at tau = 2.5: runs + figures + table
uv run build_report.py                                        # report/figures + report.zip
```

Upload `report.zip` to Overleaf with *New Project > Upload Project*, or compile
`report/main.tex` locally.

Both run scripts first run any simulations that are missing from the results
folder, then write their output. You can give the folder with `--results-dir` or
the `WTJ_RESULTS_DIR` environment variable. A run matches when its full
controller definition, bag, penetration rate, seed (7), fleet size (100) and base
controller (IDM) are the same.

From scratch, this is 708 simulations: about 1.5 hours on 8 cores and 75 GB of
per-step data. Keep the results folder outside OneDrive. If the machine crashes,
run the same command again: the damaged end of `runs.csv` is repaired, and only
the missing simulations are run.

| Script | Writes |
|---|---|
| `iteration.py` | `report/tables/iteration_path.tex`, `final_vs_alternatives.tex`, `tuning.tex` |
| `final_model.py` | `report/tables/final_tau2.5.tex`; figures in `plots/final_model/tau2.5/`: the report's `report_time_space.png`, `report_fd.png`, `report_work.png` (WTJ real drive), plus grids over every drive and penetration rate (`fundamental_diagram.png`, `cumulative_work_{STG,TG,mean}.png`) |
| `build_report.py` | copies the figures `main.tex` uses into `report/figures/`, checks every `\input` and `\includegraphics`, and zips `report/` to `report.zip` |

Useful options:
- `--dry-run` (iteration.py) lists the missing simulations.
- `--no-run` skips simulating and uses only saved runs.
- `--tau 2 3` (final_model.py) evaluates other time gaps.
- `--csv` also writes the data behind the tables and figures as CSV.

`plots/`, `report/figures/` and `report.zip` are generated and git-ignored.
`report/main.tex` and `report/tables/` are tracked.

## Controllers

All controllers are in `controllers.py`. A named controller (the `--smart` choice
of `main.py`, and the vehicle `type` in the output) is one class with fixed
parameters, listed in `PRESETS`. All use `alpha` 1, `gmin` 5 m and an acceleration
limit of [-3, 1.5] m/s².

| Name | Class | Parameters |
|---|---|---|
| `IDM` | `IDMController` | `v0` 30, `T` 1.5, `a_max` 0.73, `b` 1.67, `s0` 5: the human-driven cars |
| `TG` | `TimeGapController` | `tau` 2.5, `lam` 0.1, two-sided damping |
| `STG` | `SpecialTimeGapController` | `tau` 2.5, `lam` 2, braking-only damping: **the final controller**; identical to the saturated Simulink time gap block |
| `WTJ0` | `WTJOriginal` | `tau_min` 2, `tau_max` 3, `lam` 0.5, `c` 0.1: the original, validated against MATLAB |
| `WTJ1` | `WTJOriginal` | `lam` 2 |
| `WTJ2`-`WTJ4` | `WTJSoftLowerBound` | `beta` 0, 0.1, 0.25 (`lam` 3) |

```
STG:   a = sat( alpha (s - tau v - gmin) + lam min(0, ds/dt) )
TG:    a = sat( alpha (s - tau v - gmin) + lam ds/dt )
```

## Running the simulator directly

`main.py` runs one or more simulations and appends them to a results folder:

```sh
uv run main.py --bag wtj_real_drive --smart STG --seed 7 --penetration-rate 0.25 0.5 0.75 1 --results-dir DIR
uv run main.py --bag wtj_real_drive --smart STG --stg tau=3 lam=1    # override preset parameters
uv run summarize.py --results DIR --seed 7                           # per-bag summary tables
```

| Flag | Default | Meaning |
|---|---|---|
| `--bag NAME ...` / `--all-bags` | sine wave | Lead profile: bags in `bags/`, or a synthetic sine wave of `--length` s |
| `--smart CTRL` | `WTJ0` | Controller of the smart cars (`WTJ0`-`WTJ4`, `TG`, `STG`) |
| `--base CTRL` | `IDM` | Controller of the other followers |
| `--penetration-rate P ...` | 0.2 | Share of smart followers; several values run in parallel |
| `--n-vehicles N` | 100 | Number of followers |
| `--seed N` | random | Same seed: same smart-car positions and masses across controllers |
| `--<ctrl> KEY=VALUE ...` | | Override parameters, e.g. `--stg tau=3`, `--wtj4 beta=0.5` |
| `--jobs N` | CPU count | Simulations at once |
| `--dt DT` | 0.05 | Time step (s) |
| `--start`, `--end` | whole bag | Window of the bag to use |
| `--allow-collisions` | off | Keep simulating after a collision (default: stop and record it) |
| `--ghosts [CTRL ...]` | off | Give each smart car shadow "ghost" cars running other controllers, to compare their work in the same position |
| `--variable-mass` | off | Scale each car's acceleration response by its sampled mass |

The platoon starts at rest behind a parked leader with 50 m spacing. A run stops
at the first collision unless `--allow-collisions` is set.

Each run writes:
- `DIR/runs/<run>/vehicles.csv` and `timeseries.csv`: position, speed and
  acceleration of every vehicle at every step.
- A row in `DIR/runs.csv` with its settings, the full controller definition
  (`smart_definition`), average positive work, and safety columns:
  - `collision`
  - `min_gap_smart_m`: the closest any smart car got to the car ahead
  - `unsafe`: set when a smart car got inside 5 m
- Rows in `DIR/cars.csv`, one per car.

## Metrics

- **Work**: average positive work per follower,
  (1/N) Σᵢ (1/T) ∫ max(aᵢ vᵢ, 0) dt, in m²/s³. There is no mass term, and the
  leader is not included.
- **Flow**: Edie's flow over 250 m × 30 s boxes of the time-space plane that lie
  entirely inside the platoon, averaged over boxes, in veh/h. See
  `density_flow.py`. The box values are cached in each run folder.
- **Throughput**: vehicles per hour past the bag's marker (`MARKER_X` in
  `run_io.py`). It is a cross-check only.
- **Safety**: a collision, or a smart car closer than `gmin` = 5 m.

## Files

| File | Contents |
|---|---|
| `iteration.py` | The controller iteration: its runs and the report's iteration tables |
| `final_model.py` | STG vs TG: its runs, the report's figures and per-drive table, and full grids |
| `build_report.py` | Assembles `report/` and `report.zip` for Overleaf |
| `sweep_data.py` | Shared by both: matching runs in `runs.csv`, running missing ones, metrics |
| `summarize.py` | Per-run metrics (work, flow, throughput, safety), and per-bag summary tables |
| `report/main.tex` | The report; `report/tables/*.tex` are generated |
| `main.py` | Command-line simulator: runs simulations and writes their CSVs |
| `sim.py` | Platoon simulation |
| `controllers.py` | IDM, TG, STG, WTJOriginal, WTJSoftLowerBound, and `PRESETS` |
| `vehicles.py` | `Vehicle`, actuator dynamics, mass sampling |
| `ghosts.py` | Ghost cars (`--ghosts`) |
| `lead.py` | Lead speed profile from a bag |
| `metrics.py` | Positive work per car |
| `density_flow.py` | Box density and flow, and throughput past a marker |
| `run_io.py` | Reads and writes run folders and CSVs; bag labels and markers |
| `validation/` | Checks `WTJOriginal` and `STG` against the MATLAB/Simulink blocks (`cd validation; uv run compare_controllers.py matlab_controller_outputs.csv`). STG is the Simulink time gap block, saturated |
