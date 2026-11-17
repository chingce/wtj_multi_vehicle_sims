"""
Flow against penetration rate, one line per smart controller, in two panels:

  1. Median flow per box (density_flow.py boxes, the same values as the
     compare_flow.py box plots). Counts every part of the platoon equally.
  2. Throughput past a fixed point (default x = 2000 m): (cars that pass - 1)
     divided by the time from the first to the last of them, in veh/hr. This is
     "how many cars got through", like a loop detector would measure.

    uv run plot_flow_sweep.py --all --match realRun_n100 --defaults-only
    uv run plot_flow_sweep.py RUN [RUN ...] --marker-x 1500 --name my_sweep

Series, colors and styles come from series.py, as in plot_work_sweep.py. A p = 0
run is the shared start of every line. Each run's throughput is cached in its
folder as throughput_x<x>.csv. The plot goes to plots/flow_sweep/<name>.png.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from compare_flow import label_for, runs_csv_rows
from density_flow import DEFAULT_DT, DEFAULT_DX, density_and_flow_for
from plot_work_sweep import GRID, SURFACE, TEXT_SECONDARY, place_end_labels
from run_io import add_run_arguments, load_run, penetration_rate, resolve_runs
from series import is_secondary, panels

ROOT = Path(__file__).parent


def throughput(run_dir, x):
    """
    (veh/hr past x, cars that reached x, cars in the run), cached per run.
    Cars that never reach x are left out of the count and the time span.
    """
    cache = run_dir / f"throughput_x{x:g}.csv"
    if cache.is_file():
        with open(cache, newline="") as f:
            row = next(csv.DictReader(f))
        return float(row["veh_per_hr"]), int(row["reached"]), int(row["cars"])
    run = load_run(run_dir)
    pos = run.position[run.rows(ghost=False)][1:]          # followers only
    reached = (pos >= x).any(axis=1)
    times = np.array([run.time[np.argmax(pos[k] >= x)] for k in np.nonzero(reached)[0]])
    span = times.max() - times.min() if len(times) > 1 else np.nan
    rate = (len(times) - 1) / span * 3600 if span and span > 0 else np.nan
    with open(cache, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["veh_per_hr", "reached", "cars"])
        writer.writerow([rate, int(reached.sum()), len(pos)])
    return rate, int(reached.sum()), len(pos)


def draw_panel(ax, values, baseline, title, ylabel, notes=None, only=None):
    """values: {label: {p: value}}; baseline: value at p = 0 (shared)."""
    layout = panels(set(values), only)
    series = layout[0][1] if layout else []
    end_labels = []
    for label, color, marker in series:
        points = dict(values[label])
        if baseline is not None:
            points[0.0] = baseline
        ps = sorted(points)
        ys = [points[p] for p in ps]
        second = is_secondary(label)
        ax.plot(ps, ys, color=color, linewidth=2, solid_capstyle="round", zorder=2,
                linestyle=(0, (5, 3)) if second else "-", marker=marker, markersize=8,
                markerfacecolor=SURFACE if second else color,
                markeredgecolor=color if second else SURFACE, markeredgewidth=1.8 if second else 1.5, label=label)
        note = (notes or {}).get(label, {}).get(ps[-1], "")
        end_labels.append((ps[-1], ys[-1], f"{label}  {ys[-1]:.0f}{note}"))
    ax.set_facecolor(SURFACE)
    ax.set_xlim(-0.03, 1.32)
    ax.set_ylim(bottom=0)
    ax.set_xticks([0, 0.25, 0.5, 0.75, 1])
    ax.grid(color=GRID, linewidth=0.8, zorder=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.set_xlabel("Penetration rate (share of cars running the smart controller)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=11)
    place_end_labels(ax, end_labels)
    ax.legend(frameon=False, loc="upper right")


def main():
    parser = argparse.ArgumentParser(description="Flow vs penetration rate, by smart controller.")
    add_run_arguments(parser)
    parser.add_argument("--match", help="only runs whose name contains this text (e.g. realRun_n100)")
    parser.add_argument("--defaults-only", action="store_true",
                        help="only controllers at default parameters")
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    parser.add_argument("--marker-x", type=float, default=2000.0,
                        help="position of the throughput marker in m (default 2000)")
    parser.add_argument("--series", nargs="+", metavar="LABEL",
                        help="only these series, in one panel, e.g. TG 'TG gmin=5' WTJ3 'WTJ3 gmin=5'. "
                             "A variant takes its controller's color, drawn dashed/hollow/hatched")
    parser.add_argument("--name", help="output file name (default: built from --match and settings)")
    args = parser.parse_args()

    run_dirs = resolve_runs(args.runs, all_runs=args.all or bool(args.match and not args.runs))
    if args.match:
        run_dirs = [d for d in run_dirs if args.match in d.name]
    if not run_dirs:
        parser.error("no runs selected")
    rows = runs_csv_rows(run_dirs[0].parent.parent)
    if args.defaults_only:
        run_dirs = [d for d in run_dirs
                    if (rows.get(d.name, {}).get("controller_params") or "defaults") == "defaults"]
    if args.series:   # the chosen series, plus the shared p = 0 baseline
        run_dirs = [d for d in run_dirs if penetration_rate(d.name) == 0 or label_for(d, rows) in args.series]

    box_median, through = defaultdict(dict), defaultdict(dict)
    notes = defaultdict(dict)
    base_box = base_through = None
    for run_dir in sorted(run_dirs, key=lambda d: (penetration_rate(d.name), d.name)):
        p = penetration_rate(run_dir.name)
        label = label_for(run_dir, rows)
        med = float(np.median(density_and_flow_for(run_dir, args.dx, args.dT)["flow"]))
        rate, reached, cars = throughput(run_dir, args.marker_x)
        print(f"[{run_dir.name}] p={p:g} {label}: median box flow {med:.0f} veh/hr, "
              f"throughput {rate:.0f} veh/hr ({reached}/{cars} cars reached {args.marker_x:g} m)")
        if p == 0:
            base_box, base_through = med, rate
        else:
            box_median[label][p] = med
            through[label][p] = rate
            if reached < cars:
                notes[label][p] = f" ({cars - reached} short)"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.5), facecolor=SURFACE)
    draw_panel(ax1, box_median, base_box,
               f"Median flow per box ({args.dx:g} m x {args.dT:g} s, inside the platoon)", "Flow (veh/hr)",
               only=args.series)
    draw_panel(ax2, through, base_through,
               f"Throughput past x = {args.marker_x:g} m (cars through / time, first to last)",
               "Flow (veh/hr)", notes, only=args.series)
    # Same y range on both panels, tall enough for every point (including the p = 0 baseline)
    top = max(ax1.get_ylim()[1], ax2.get_ylim()[1],
              max([v for d in (box_median, through) for vals in d.values() for v in vals.values()]
                  + [b for b in (base_box, base_through) if b is not None]) * 1.05)
    for ax in (ax1, ax2):
        ax.set_ylim(0, top)
    sources = sorted({rows.get(d.name, {}).get("source", "") for d in run_dirs} - {""})
    fig.suptitle(f"Flow vs penetration rate: {args.match or ', '.join(sources) or 'selected runs'}; {len(run_dirs)} runs",
                 x=0.01, ha="left", fontsize=12)
    fig.tight_layout()

    name = args.name or f"{args.match or 'runs'}_dx{args.dx:g}_dT{args.dT:g}_x{args.marker_x:g}"
    out_dir = ROOT / "plots" / "flow_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
    print(f"Flow sweep plot: {path}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
