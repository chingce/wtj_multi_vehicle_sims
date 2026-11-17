"""
Average positive work against penetration rate, read from results/runs.csv.

    uv run plot_work_sweep.py --source realRun
    uv run plot_work_sweep.py --source realRun --base IDM --seed 7
    uv run plot_work_sweep.py --results variable_mass_results/results/runs.csv --source realRun

Panels (see series.py): the smart controllers at default parameters (WTJ,
WTJ3, WTJ4, TG), then one panel per controller that also has runs with changed
parameters (e.g. "WTJ3 gmin=5"), with its default line repeated for reference.

Each point is the mean over the matching runs at that penetration rate (small
dots show the individual runs when there is more than one). A p = 0 run has no
smart cars, so it is the shared starting point of every line. A run stopped by
a collision has no work value: it is drawn as an x on the x-axis, and the line
breaks there. Only rows written since runs.csv got the smart_controller column
are used.

The plot goes to plots/work_sweep/<name>.png.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from series import is_secondary, panels, series_label

ROOT = Path(__file__).parent

SURFACE = "#fcfcfb"
GRID = "#e4e3df"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"


def load_rows(path, source, base, n_vehicles, seed):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    keep = []
    for r in rows:
        if not r.get("smart_controller"):
            continue                      # written before runs.csv recorded it
        if source and r["source"] != source:
            continue
        if base and r.get("base_controller") != base:
            continue
        if n_vehicles and int(r["n_vehicles"]) != n_vehicles:
            continue
        if seed is not None and r.get("seed") != str(seed):
            continue
        keep.append(r)
    return keep


def place_end_labels(ax, labels, min_gap=0.045):
    """
    Direct labels at line ends. Labels closer than min_gap (fraction of the y
    range) are pushed apart, with a thin leader line back to the point.
    """
    top = max([y for _, y, _ in labels] + [ax.get_ylim()[1]])
    gap = min_gap * top
    placed = []
    for x, y, text in sorted(labels, key=lambda t: t[1]):
        ty = y
        for _, py in placed:
            if abs(ty - py) < gap:
                ty = py + gap
        placed.append((x, ty))
        ax.annotate(text, (x, y), xytext=(x + 0.04, ty), textcoords="data", va="center",
                    color=TEXT, fontsize=9.5,
                    arrowprops=None if ty == y else dict(arrowstyle="-", color=TEXT_SECONDARY, linewidth=0.6))


def plot_sweep(rows, title, only=None):
    work = defaultdict(lambda: defaultdict(list))   # label -> p -> [avg work]
    crashed = defaultdict(set)                       # label -> {p} stopped by a collision
    baseline = []                                     # p = 0 runs, shared by all lines
    for r in rows:
        p = float(r["penetration_rate"])
        if r.get("collision"):
            crashed[series_label(r)].add(p)
        elif p == 0:
            baseline.append(float(r["avg_positive_work"]))
        else:
            work[series_label(r)][p].append(float(r["avg_positive_work"]))

    layout = panels(set(work) | set(crashed), only)
    fig, axes = plt.subplots(1, len(layout), figsize=(7.5 * len(layout), 6), sharey=True,
                             facecolor=SURFACE, squeeze=False)
    for ax, (panel_title, series) in zip(axes[0], layout):
        ax.set_facecolor(SURFACE)
        end_labels = []   # (x, y, text), placed after all lines so they can be kept apart
        for j, (label, color, marker) in enumerate(series):
            # Spread each series' x marks a little sideways so marks at the same p don't hide each other
            dodge = (j - (len(series) - 1) / 2) * 0.03
            if not work[label]:
                # Every run of this series was stopped: show only the x marks, not the baseline
                ax.scatter([p + dodge for p in sorted(crashed[label])], [0] * len(crashed[label]), marker="x",
                           s=70, color=color, linewidths=2.2, zorder=4, clip_on=False, label=label)
                continue
            points = dict(work[label])
            if baseline:
                points[0.0] = baseline
            grid = sorted(set(points) | crashed[label])
            ys = [np.mean(points[p]) if p in points else np.nan for p in grid]   # NaN breaks the line
            second = is_secondary(label)   # changed parameters: dashed line, hollow marker
            ax.plot(grid, ys, color=color, linewidth=2, solid_capstyle="round", zorder=2,
                    linestyle=(0, (5, 3)) if second else "-", marker=marker, markersize=8,
                    markerfacecolor=SURFACE if second else color,
                    markeredgecolor=color if second else SURFACE, markeredgewidth=1.8 if second else 1.5,
                    label=label)
            for p in points:                   # individual runs, when repeated
                if len(points[p]) > 1:
                    ax.scatter([p] * len(points[p]), points[p], s=14, color=color, alpha=0.5, zorder=2)
            if crashed[label]:
                ax.scatter([p + dodge for p in sorted(crashed[label])], [0] * len(crashed[label]), marker="x", s=70,
                           color=color, linewidths=2.2, zorder=4, clip_on=False)
            last = max(p for p in points)
            end_labels.append((last, np.mean(points[last]), f"{label}  {np.mean(points[last]):.3f}"))
        place_end_labels(ax, end_labels)

        ax.set_xlim(-0.03, 1.3)
        ax.set_ylim(bottom=0)
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1])
        ax.grid(color=GRID, linewidth=0.8, zorder=0)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=TEXT_SECONDARY)
        ax.set_xlabel("Penetration rate (share of cars running the smart controller)")
        ax.set_title(panel_title, loc="left", fontsize=11)
        handles, _ = ax.get_legend_handles_labels()
        if any(crashed[label] for label, *_ in series):
            handles.append(Line2D([], [], marker="x", linestyle="", color=TEXT_SECONDARY, markersize=8,
                                  markeredgewidth=2, label="stopped by a collision"))
        ax.legend(handles=handles, frameon=False, loc="upper right")
    axes[0][0].set_ylabel("Average positive work, all followers (m²/s³)")
    fig.suptitle(title, x=0.01, ha="left", fontsize=12)
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Average positive work vs penetration rate, by smart controller.")
    parser.add_argument("--results", default=str(ROOT / "results" / "runs.csv"), help="runs.csv to read")
    parser.add_argument("--source", help="bag name (or 'sine') to keep, e.g. realRun")
    parser.add_argument("--base", help="only runs with this base controller (IDM or TG)")
    parser.add_argument("--n-vehicles", type=int, help="only runs with this many followers")
    parser.add_argument("--seed", type=int, help="only runs made with this --seed")
    parser.add_argument("--defaults-only", action="store_true",
                        help="only controllers at default parameters (no variants panel)")
    parser.add_argument("--series", nargs="+", metavar="LABEL",
                        help="only these series, in one panel, e.g. TG 'TG gmin=5' WTJ3 'WTJ3 gmin=5'. "
                             "A variant takes its controller's color, drawn dashed/hollow/hatched")
    parser.add_argument("--name", help="output file name (default: built from the filters)")
    parser.add_argument("--show", action="store_true", help="also open the plot in a window")
    args = parser.parse_args()

    rows = load_rows(args.results, args.source, args.base, args.n_vehicles, args.seed)
    if args.defaults_only:
        rows = [r for r in rows if (r.get("controller_params") or "defaults") == "defaults"]
    if args.series:   # the chosen series, plus the shared p = 0 baseline
        rows = [r for r in rows if float(r["penetration_rate"]) == 0 or series_label(r) in args.series]
    if not rows:
        parser.error(f"no matching runs with a smart_controller column in {args.results}")
    for r in rows:
        status = f"STOPPED ({r['collision']})" if r.get("collision") else f"avg work {float(r['avg_positive_work']):.4f}"
        print(f"  {r['run']}: p={float(r['penetration_rate']):g} {series_label(r)}: {status}")

    filters = [args.source or "all", args.base, f"n{args.n_vehicles}" if args.n_vehicles else None,
               f"seed{args.seed}" if args.seed is not None else None]
    name = args.name or "_".join(f for f in filters if f)
    others = {r.get("base_controller") for r in rows}
    n_stopped = sum(bool(r.get("collision")) for r in rows)
    title = (f"Average positive work vs penetration rate: {args.source or 'all sources'}, "
             f"other cars {', '.join(sorted(o for o in others if o))}; "
             f"{len(rows) - n_stopped} runs" + (f", {n_stopped} stopped by a collision" if n_stopped else ""))

    out_dir = Path(args.results).resolve().parent.parent / "plots" / "work_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.png"
    plot_sweep(rows, title, args.series).savefig(path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
    print(f"Work sweep plot: {path}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
