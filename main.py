import argparse
import csv
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

from controllers import CONTROLLERS, controller_param_names
from ghosts import DEFAULT_GHOSTS, compare_ghosts
from lead import load_lead_from_bag
from metrics import positive_work_per_car
from sim import sim

ROOT = Path(__file__).parent
BAGS_DIR = ROOT / "bags"
VARIABLE_MASS_OUTPUT_DIR = ROOT / "variable_mass_results"
PLOTS_DIR = ROOT / "plots"
RESULTS_DIR = ROOT / "results"

TYPE_COLORS = {"lead": "green", "WTJ": "blue", "TG": "red"}


def vehicle_color(i, platoon):
    """Color for row i of the sim output (row 0 is the leader)."""
    return TYPE_COLORS["lead"] if i == 0 else TYPE_COLORS[platoon[i - 1].type]


def save_figure(fig, graph, run_name):
    """Save a figure to plots/<graph>/<run_name>.png."""
    out_dir = PLOTS_DIR / graph
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{run_name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    return path


def plot_state(data, time, platoon, index, ylabel, title):
    """One line per vehicle for one state (0 position, 1 speed, 2 acceleration)."""
    fig, ax = plt.subplots(figsize=(10, 6))
    for i in range(data.shape[0]):
        ax.plot(time, data[i, index, :], color=vehicle_color(i, platoon), linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    return fig


BAR_ALPHA = 0.6     # Opacity of the real cars' work bars
GHOST_ALPHA = 0.2   # Opacity of the ghost work bars drawn over them


def plot_work(work, platoon, title, ghosts=(), ghost_types=(), duration=None):
    """
    Bar per follower, colored by controller type. Ghosts, if any, are drawn as
    fainter bars in the same slot as their WTJ car's bar, using the same
    average-rate measure (ghost total work / sim length).
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    car_ids = [v.id for v in platoon]
    ax.bar(car_ids, work, color=[TYPE_COLORS[v.type] for v in platoon], alpha=BAR_ALPHA, zorder=2)
    ax.axhline(work.mean(), color="black", linestyle="--", linewidth=1,
               label=f"mean {work.mean():.4f}")
    handles = [Patch(color=TYPE_COLORS["WTJ"], alpha=BAR_ALPHA, label="WTJ"),
               Patch(color=TYPE_COLORS["TG"], alpha=BAR_ALPHA, label="TG"),
               ax.get_lines()[0]]

    for ghost_type in ghost_types:
        mine = [r for r in ghosts if r.ghost_type == ghost_type]
        color = TYPE_COLORS.get(ghost_type, "gray")
        ax.bar([r.car for r in mine], [r.ghost_total_work / duration for r in mine],
               color=color, alpha=GHOST_ALPHA, zorder=1)  # behind the real bars
        handles.append(Patch(color=color, alpha=GHOST_ALPHA, label=f"{ghost_type} ghost on WTJ car"))

    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("Car (position behind leader)")
    ax.set_ylabel("Average positive work (m²/s³)")
    ax.set_title(title)
    ax.legend(handles=handles)
    return fig


def append_rows(path, fieldnames, rows):
    """
    Append rows to a CSV, writing the header if the file is new. If the file's
    header lacks some of `fieldnames` (a column was added since it was written),
    the file is rewritten with the new columns, left blank for the old rows.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
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
    else:
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def format_params(controller_params):
    """{"WTJ": {"c": 0.2}} -> "WTJ c=0.2" (empty string for defaults)."""
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


def run_one(args, bag_path, stamp, show):
    lead_vel = None
    if bag_path is not None:
        lead_vel = load_lead_from_bag(bag_path, args.dt, topic=args.bag_topic,
                                      start=args.start, end=args.end)

    data, time, platoon = sim(
        n_vehicles=args.n_vehicles,
        penetration_rate=args.penetration_rate,
        sim_length_seconds=args.length,
        dt=args.dt,
        lead_vel=lead_vel,
        ghosts=args.ghosts,
        controller_params=args.controller_params,
        variable_mass=args.variable_mass,
    )

    source = bag_path.stem if bag_path is not None else "sine"
    params = format_params(args.controller_params)
    # e.g. "_WTJ-c0.2" when WTJ's c was changed; nothing when all defaults
    param_tag = "".join(f"_{name}-" + "-".join(f"{k}{v:g}" for k, v in p.items())
                        for name, p in args.controller_params.items() if p)
    run_name = f"{stamp}_{source}_n{args.n_vehicles}_p{args.penetration_rate:g}{param_tag}"
    duration = len(time) * args.dt
    work = positive_work_per_car(data, args.dt)
    n_wtj = sum(v.type == "WTJ" for v in platoon)

    print(f"[{run_name}] {len(platoon)} vehicles, {duration:g}s "
          f"({n_wtj} WTJ, {len(platoon) - n_wtj} TG), "
          f"{'variable' if args.variable_mass else 'fixed'} mass dynamics, "
          f"avg positive work {work.mean():.5f} m^2/s^3")

    append_rows(
        RESULTS_DIR / "runs.csv",
        ["run", "source", "n_vehicles", "penetration_rate", "n_wtj", "n_tg",
         "duration_s", "dt", "variable_mass", "avg_positive_work", "controller_params"],
        [{"run": run_name, "source": source, "n_vehicles": args.n_vehicles,
          "penetration_rate": args.penetration_rate, "n_wtj": n_wtj,
          "n_tg": len(platoon) - n_wtj, "duration_s": duration, "dt": args.dt,
          "variable_mass": args.variable_mass,
          "avg_positive_work": work.mean(), "controller_params": params or "defaults"}],
    )
    append_rows(
        RESULTS_DIR / "cars.csv",
        ["run", "car", "type", "mass_kg", "variable_mass", "avg_positive_work"],
        [{"run": run_name, "car": v.id, "type": v.type, "mass_kg": round(v.mass, 1),
          "variable_mass": args.variable_mass,
          "avg_positive_work": w} for v, w in zip(platoon, work)],
    )

    ghosts = compare_ghosts(data, platoon, args.dt)
    if ghosts:
        for ghost_type in args.ghosts:
            diffs = np.array([r.work_diff for r in ghosts if r.ghost_type == ghost_type])
            print(f"    {ghost_type} ghosts on {len(diffs)} WTJ cars: ghost - WTJ total work "
                  f"sum {diffs.sum():+.2f}, mean {diffs.mean():+.3f} m^2/s^2 per car "
                  f"(positive = {ghost_type} would have done more work)")
        rows = [{"run": run_name, "variable_mass": args.variable_mass, **asdict(r)} for r in ghosts]
        append_rows(RESULTS_DIR / "ghosts.csv", list(rows[0]), rows)

    if args.no_plot:
        return

    title = f"{source}, n={args.n_vehicles}, p={args.penetration_rate:g}"
    title += ", variable mass" if args.variable_mass else ", fixed mass"
    title += f", {params}" if params else ""
    figures = {
        "positions": plot_state(data, time, platoon, 0, "Position (m)", title),
        "speeds": plot_state(data, time, platoon, 1, "Speed (m/s)", title),
        "accelerations": plot_state(data, time, platoon, 2, "Acceleration (m/s²)", title),
        "positive_work": plot_work(work, platoon, title, ghosts=ghosts,
                                   ghost_types=args.ghosts or (), duration=duration),
    }
    for graph, fig in figures.items():
        save_figure(fig, graph, run_name)
    if show:
        plt.show()
    plt.close("all")


def main():
    parser = argparse.ArgumentParser(description="Run the multi-vehicle traffic sim.")
    parser.add_argument("--n-vehicles", type=int, default=100)
    parser.add_argument("--penetration-rate", type=float, default=0.2)
    parser.add_argument("--length", type=float, default=360,
                        help="sim length in seconds (synthetic lead only)")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--variable-mass", action="store_true",
                        help="scale each vehicle's acceleration response by its sampled mass; "
                             "outputs go to variable_mass_results/")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--bag", help="bag to take the lead speed from: a path, or a name in bags/")
    source.add_argument("--all-bags", action="store_true", help="run once per .bag in bags/")
    parser.add_argument("--bag-topic", help="speed topic in the bag (default: auto-detect)")
    parser.add_argument("--start", type=float, default=0.0,
                        help="seconds into the bag to start the lead profile")
    parser.add_argument("--end", type=float, help="seconds into the bag to end the lead profile")
    parser.add_argument("--ghosts", nargs="*", choices=list(CONTROLLERS), metavar="CTRL",
                        help="give every WTJ car a ghost for each listed controller "
                             f"({', '.join(CONTROLLERS)}); with no list, one "
                             f"{' + '.join(DEFAULT_GHOSTS)} ghost. Off unless given.")
    for name in CONTROLLERS:
        parser.add_argument(f"--{name.lower()}", nargs="+", metavar="KEY=VALUE",
                            help=f"override {name} controller parameters "
                                 f"({', '.join(controller_param_names(name))}); also applies to {name} ghosts")
    parser.add_argument("--no-plot", action="store_true", help="skip saving plots")
    parser.add_argument("--show", action="store_true", help="also open the plots in a window")
    args = parser.parse_args()

    global PLOTS_DIR, RESULTS_DIR
    if args.variable_mass:
        PLOTS_DIR = VARIABLE_MASS_OUTPUT_DIR / "plots"
        RESULTS_DIR = VARIABLE_MASS_OUTPUT_DIR / "results"

    args.controller_params = {name: parse_params(name, getattr(args, name.lower()), parser)
                              for name in CONTROLLERS}

    if args.ghosts == []:  # flag given with no controllers
        args.ghosts = list(DEFAULT_GHOSTS)
    elif args.ghosts:
        args.ghosts = list(dict.fromkeys(args.ghosts))  # drop repeats, keep order

    if args.all_bags:
        bags = sorted(BAGS_DIR.glob("*.bag"))
        if not bags:
            parser.error(f"No .bag files in {BAGS_DIR}")
    elif args.bag:
        bags = [resolve_bag(args.bag)]
    else:
        bags = [None]  # synthetic sine-wave lead

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    for bag_path in bags:
        run_one(args, bag_path, stamp, show=args.show and len(bags) == 1)

    print(f"Results appended to {RESULTS_DIR}; plots in {PLOTS_DIR}")


if __name__ == "__main__":
    main()
