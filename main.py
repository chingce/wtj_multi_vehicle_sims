import argparse
import csv
import os
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np

from controllers import (COMPARISON_CONTROLLERS, CONTROLLERS, SMART_CONTROLLERS, controller_param_names,
                         preset_label)
from ghosts import DEFAULT_GHOSTS, compare_ghosts
from lead import load_lead_from_bag
from metrics import positive_work_per_car
from run_io import repair_csv, write_run_csvs
from sim import CollisionError, sim

ROOT = Path(__file__).parent
BAGS_DIR = ROOT / "bags"
VARIABLE_MASS_OUTPUT_DIR = ROOT / "variable_mass_results"
RESULTS_DIR = ROOT / "results"
SAFE_GAP = 5.0       # m: a smart car closer than this to the car ahead marks the run unsafe
GAP_TOLERANCE = 0.01 # m: WTJ settles at exactly gmin = 5 m; don't flag rounding at that edge


def gap_summary(data, platoon, smart):
    """
    Closest bumper-to-bumper approach over the run. Returns
    (min gap of any follower, its car type, min gap of a `smart` car or None).
    """
    gaps = data[:-1, 0, :] - data[1:, 0, :]     # row i: follower i+1's gap to the car ahead
    per_car = gaps.min(axis=1)
    worst = int(per_car.argmin())
    smart_rows = [i for i, v in enumerate(platoon) if v.type == smart]
    smart_min = float(per_car[smart_rows].min()) if smart_rows else None
    return float(per_car[worst]), platoon[worst].type, smart_min

def append_rows(path, fieldnames, rows):
    """
    Append rows to a CSV, writing the header if the file is new. If the file's
    header lacks some of `fieldnames` (a column was added since it was written),
    the file is rewritten with the new columns, left blank for the old rows.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    repair_csv(path)          # a crash during an earlier append can leave NUL bytes or a cut-off line
    if path.exists():
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            old_fields = reader.fieldnames or []
            missing = [name for name in fieldnames if name not in old_fields]
            old_rows = list(reader) if missing else None
        if missing:
            fieldnames = old_fields + missing
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(old_rows)
        else:
            fieldnames = old_fields
        with open(path, "a", newline="") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writerows(rows)
            f.flush()
            os.fsync(f.fileno())
    else:
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def format_params(controller_params):
    """{"WTJ3": {"c": 0.2}} -> "WTJ3 c=0.2" (empty string for defaults)."""
    return "; ".join(f"{name} " + " ".join(f"{k}={v:g}" for k, v in params.items())
                     for name, params in controller_params.items() if params)


def parse_params(controller, pairs, parser):
    """Turn ["c=0.2", "tau_min=1.5"] into {"c": 0.2, "tau_min": 1.5}, checking names."""
    params = {}
    valid = controller_param_names(controller)
    for pair in pairs or []:
        key, sep, value = pair.partition("=")
        if not sep:
            parser.error(f"--{controller.lower()} expects key=value, got {pair!r}")
        if key not in valid:
            parser.error(f"{controller} controller has no parameter {key!r}; it takes {', '.join(valid)}")
        try:
            params[key] = float(value)
        except ValueError:
            parser.error(f"--{controller.lower()} {key}: {value!r} is not a number")
    return params


def resolve_bag(name):
    """Accept a path, or a file name (with or without .bag) inside bags/."""
    for candidate in (Path(name), BAGS_DIR / name, BAGS_DIR / f"{name}.bag"):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"Could not find bag {name!r} (also looked in {BAGS_DIR}).")


def run_sim(args, lead_vel, rate):
    return sim(
        n_vehicles=args.n_vehicles,
        penetration_rate=rate,
        sim_length_seconds=args.length,
        dt=args.dt,
        lead_vel=lead_vel,
        ghosts=args.ghosts,
        controller_params=args.controller_params,
        variable_mass=args.variable_mass,
        base_controller=args.base,
        smart_controller=args.smart,
        stop_on_collision=not args.allow_collisions,
    )


def run_one(args, bag_path, stamp, rate, results_dir):
    """
    One simulation at penetration rate `rate`. Safe to run in a worker process:
    it writes only its own run folder, and returns (lines to print, rows to add
    to the shared CSVs) so the main process does all writes to runs.csv,
    cars.csv and ghosts.csv.
    """
    lines, appends = [], []   # appends: (csv file name, fieldnames, rows)
    if args.seed is not None:
        # Same seed -> same controller draw and masses, so runs that differ only in
        # --smart (or a controller parameter) put the same kind of car in each slot
        random.seed(args.seed)
        np.random.seed(args.seed)

    lead_vel = None
    if bag_path is not None:
        lead_vel = load_lead_from_bag(bag_path, args.dt, topic=args.bag_topic,
                                      start=args.start, end=args.end)

    source = bag_path.stem if bag_path is not None else "sine"
    params = format_params(args.controller_params)
    # e.g. "_WTJ-c0.2" when WTJ's c was changed; nothing when all defaults
    param_tag = "".join(f"_{name}-" + "-".join(f"{k}{v:g}" for k, v in p.items())
                        for name, p in args.controller_params.items() if p)
    smart_tag = f"_{args.smart}"   # e.g. "_WTJ3"
    run_name = f"{stamp}_{source}_n{args.n_vehicles}_p{rate:g}_{args.base}{smart_tag}{param_tag}"
    run_fields = ["run", "source", "n_vehicles", "penetration_rate", "smart_controller", "base_controller",
                  "n_wtj", "n_base", "duration_s", "dt", "variable_mass", "avg_positive_work",
                  "controller_params", "seed", "collision", "unsafe", "min_gap_smart_m", "min_gap_m",
                  "min_gap_type", "smart_definition"]
    run_row = {"run": run_name, "source": source, "n_vehicles": args.n_vehicles,
               "penetration_rate": rate, "smart_controller": args.smart,
               "base_controller": args.base, "dt": args.dt, "variable_mass": args.variable_mass,
               "controller_params": params or "defaults",
               "seed": "" if args.seed is None else args.seed, "collision": "",
               "smart_definition": preset_label(args.smart, args.controller_params.get(args.smart))}

    try:
        data, time, platoon = run_sim(args, lead_vel, rate)
    except CollisionError as crash:
        # The run is stopped: record what happened in runs.csv, and write nothing else
        lines.append(f"[{run_name}] STOPPED: {crash}")
        appends.append(("runs.csv", run_fields, [{**run_row, "collision": str(crash),
                                                  "unsafe": f"collision ({crash.car_type} car {crash.car})"}]))
        return lines, appends

    duration = len(time) * args.dt
    work = positive_work_per_car(data, args.dt)
    n_wtj = sum(v.type == args.smart for v in platoon)
    n_base = len(platoon) - n_wtj
    min_gap, min_gap_type, smart_gap = gap_summary(data, platoon, args.smart)
    unsafe = ""
    if smart_gap is not None and smart_gap < SAFE_GAP - GAP_TOLERANCE:
        unsafe = f"{args.smart} car within {smart_gap:.2f} m (< {SAFE_GAP:g} m)"

    lines.append(f"[{run_name}] {len(platoon)} vehicles, {duration:g}s "
                 f"({n_wtj} {args.smart}, {n_base} {args.base}), "
                 f"{'variable' if args.variable_mass else 'fixed'} mass dynamics, "
                 f"avg positive work {work.mean():.5f} m^2/s^3, closest gap {min_gap:.2f} m ({min_gap_type})"
                 + (f"  ** UNSAFE: {unsafe}" if unsafe else ""))
    appends.append(("runs.csv", run_fields,
                    [{**run_row, "n_wtj": n_wtj, "n_base": n_base, "duration_s": duration,
                      "avg_positive_work": work.mean(), "unsafe": unsafe,
                      "min_gap_smart_m": "" if smart_gap is None else round(smart_gap, 3),
                      "min_gap_m": round(min_gap, 3), "min_gap_type": min_gap_type}]))
    appends.append(("cars.csv", ["run", "car", "type", "mass_kg", "variable_mass", "avg_positive_work"],
                    [{"run": run_name, "car": v.id, "type": v.type, "mass_kg": round(v.mass, 1),
                      "variable_mass": args.variable_mass, "avg_positive_work": w}
                     for v, w in zip(platoon, work)]))

    run_dir = results_dir / "runs" / run_name
    write_run_csvs(run_dir, data, time, platoon)
    lines.append(f"    per-vehicle CSVs in {run_dir}")

    ghosts = compare_ghosts(data, platoon, args.dt)
    if ghosts:
        for ghost_type in args.ghosts:
            diffs = np.array([r.work_diff for r in ghosts if r.ghost_type == ghost_type])
            lines.append(f"    {ghost_type} ghosts on {len(diffs)} {args.smart} cars: ghost - {args.smart} total work "
                         f"sum {diffs.sum():+.2f}, mean {diffs.mean():+.3f} m^2/s^2 per car "
                         f"(positive = {ghost_type} would have done more work)")
        rows = [{"run": run_name, "variable_mass": args.variable_mass, **asdict(r)} for r in ghosts]
        appends.append(("ghosts.csv", list(rows[0]), rows))
    return lines, appends


def main():
    parser = argparse.ArgumentParser(description="Run the multi-vehicle traffic sim.")
    parser.add_argument("--n-vehicles", type=int, default=100)
    parser.add_argument("--allow-collisions", action="store_true",
                        help="keep simulating when a car reaches the car ahead (default: stop the run "
                             "and record the collision in runs.csv)")
    parser.add_argument("--seed", type=int,
                        help="random seed for controller assignment and masses (default: random each run)")
    parser.add_argument("--penetration-rate", type=float, nargs="+", default=[0.2], metavar="P",
                        help="share of followers running --smart (0 to 1). Give several values to run "
                             "them all, in parallel (see --jobs)")
    parser.add_argument("--jobs", type=int, default=0,
                        help="how many simulations to run at once when there are several "
                             "(rates x bags); default: all of them, up to the CPU count. 1 = one at a time")
    parser.add_argument("--smart", choices=list(SMART_CONTROLLERS) + list(COMPARISON_CONTROLLERS), default="WTJ0",
                        help="controller run by the --penetration-rate share of vehicles (default WTJ0; "
                             "see controllers.PRESETS for what each WTJ0-WTJ4 is). "
                             "TG and STG are allowed for comparison; they get no ghosts")
    parser.add_argument("--base", choices=[c for c in CONTROLLERS if c not in SMART_CONTROLLERS], default="IDM",
                        help="controller for the vehicles that don't run --smart (default IDM)")
    parser.add_argument("--length", type=float, default=360,
                        help="sim length in seconds (synthetic lead only)")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--results-dir",
                        help="folder for runs.csv, cars.csv, ghosts.csv and runs/ (default results/, or "
                             "variable_mass_results/results/ with --variable-mass). Use a folder outside "
                             "OneDrive for big sweeps: each full-bag run writes ~100-200 MB")
    parser.add_argument("--variable-mass", action="store_true",
                        help="scale each vehicle's acceleration response by its sampled mass; "
                             "outputs go to variable_mass_results/results/")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--bag", nargs="+", metavar="BAG",
                        help="bag(s) to take the lead speed from: paths, or names in bags/. "
                             "Several bags run in parallel like several rates")
    source.add_argument("--all-bags", action="store_true", help="run once per .bag in bags/")
    parser.add_argument("--bag-topic", help="speed topic in the bag (default: auto-detect)")
    parser.add_argument("--start", type=float, default=0.0,
                        help="seconds into the bag to start the lead profile")
    parser.add_argument("--end", type=float, help="seconds into the bag to end the lead profile")
    parser.add_argument("--ghosts", nargs="*", choices=list(CONTROLLERS), metavar="CTRL",
                        help="give every smart car a ghost for each listed controller "
                             f"({', '.join(CONTROLLERS)}); with no list, one "
                             f"{' + '.join(DEFAULT_GHOSTS)} ghost. Off unless given.")
    for name in CONTROLLERS:
        parser.add_argument(f"--{name.lower()}", nargs="+", metavar="KEY=VALUE",
                            help=f"override {name} controller parameters "
                                 f"({', '.join(controller_param_names(name))}); also applies to {name} ghosts")
    args = parser.parse_args()

    global RESULTS_DIR
    if args.variable_mass:
        RESULTS_DIR = VARIABLE_MASS_OUTPUT_DIR / "results"
    if args.results_dir:
        RESULTS_DIR = Path(args.results_dir)

    args.controller_params = {name: parse_params(name, getattr(args, name.lower()), parser)
                              for name in CONTROLLERS}

    if args.smart == args.base:
        parser.error(f"--smart and --base are both {args.base}; pick different controllers")

    if args.ghosts == []:  # flag given with no controllers
        args.ghosts = list(DEFAULT_GHOSTS)
    elif args.ghosts:
        args.ghosts = list(dict.fromkeys(args.ghosts))  # drop repeats, keep order

    if args.all_bags:
        bags = sorted(BAGS_DIR.glob("*.bag"))
        if not bags:
            parser.error(f"No .bag files in {BAGS_DIR}")
    elif args.bag:
        bags = [resolve_bag(name) for name in args.bag]
    else:
        bags = [None]  # synthetic sine-wave lead

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    tasks = [(bag_path, rate) for bag_path in bags for rate in args.penetration_rate]
    jobs = min(args.jobs or os.cpu_count() or 1, len(tasks))

    def record(lines, appends):
        # Only the main process writes the shared CSVs, one run at a time
        for line in lines:
            print(line, flush=True)
        for file_name, fields, rows in appends:
            append_rows(RESULTS_DIR / file_name, fields, rows)

    if jobs <= 1:
        for bag_path, rate in tasks:
            record(*run_one(args, bag_path, stamp, rate, RESULTS_DIR))
    else:
        print(f"Running {len(tasks)} simulations, {jobs} at a time...", flush=True)
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            futures = [pool.submit(run_one, args, bag_path, stamp, rate, RESULTS_DIR) for bag_path, rate in tasks]
            for future in as_completed(futures):
                record(*future.result())

    print(f"Results in {RESULTS_DIR}. To plot: uv run plot_run.py (and plot_fd.py) [RUN ...]")


if __name__ == "__main__":
    main()
