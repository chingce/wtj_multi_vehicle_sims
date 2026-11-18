"""
One fundamental diagram (flow vs density) for a set of runs, with each point
colored by the penetration rate of the run it came from.

Each point is one box from density_flow.py. Box values are read from a run's
density_flow_dx<dx>_dT<dT>.csv when it exists, and computed and saved otherwise.

    uv run plot_fd.py --all --match wtj_real_drive          # every wtj_real_drive run, one diagram
    uv run plot_fd.py RUN_A RUN_B RUN_C              # chosen runs
    uv run plot_fd.py --all --match wtj_real_drive --dx 500 --dT 60 --name wtj_real_drive_sweep

--match keeps only runs whose name contains that text; use it so one diagram
doesn't mix different bags or controller settings. The plot goes to
plots/fundamental_diagram/<name>.png.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from density_flow import DEFAULT_DT, DEFAULT_DX, density_and_flow_for
from run_io import add_run_arguments, penetration_rate, resolve_runs

ROOT = Path(__file__).parent

# Ordinal blue ramp, light -> dark (dataviz reference palette, steps 250-700).
# Lighter = lower penetration rate.
BLUE_RAMP = ["#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6",
             "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]
SURFACE = "#fcfcfb"
GRID = "#e4e3df"
TEXT_SECONDARY = "#52514e"


def ramp_colors(n):
    """n colors spread evenly over the blue ramp (one level gets the middle step)."""
    if n == 1:
        return [BLUE_RAMP[4]]
    return [BLUE_RAMP[i] for i in np.linspace(0, len(BLUE_RAMP) - 1, n).round().astype(int)]


def plot_fundamental_diagram(points, dx, dT, title):
    """
    points: {penetration rate: (density array, flow array, number of runs)}.
    """
    levels = sorted(points)
    colors = dict(zip(levels, ramp_colors(len(levels))))

    fig, ax = plt.subplots(figsize=(9, 6.5), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for p in levels:   # low penetration first, so higher rates draw on top
        density, flow, _ = points[p]
        ax.scatter(density, flow, s=26, color=colors[p], edgecolors=SURFACE, linewidths=0.8, zorder=2)

    # Start the axes at 0, unless a box has negative flow (cars moving backward) - never hide it
    lowest_flow = min(points[p][1].min() for p in levels)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=min(0, lowest_flow * 1.05))
    if lowest_flow < 0:
        ax.axhline(0, color=TEXT_SECONDARY, linewidth=0.8, zorder=1)
    ax.grid(color=GRID, linewidth=0.8, zorder=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.set_xlabel("Density (veh/km)")
    ax.set_ylabel("Flow (veh/hr)")
    ax.set_title(f"{title}\nboxes of {dx:g} m x {dT:g} s", loc="left")

    handles = [Line2D([], [], marker="o", linestyle="", markersize=8, color=colors[p],
                      markeredgecolor=SURFACE, label=f"{p:g}  ({len(points[p][0])} boxes, "
                                                     f"{points[p][2]} run{'s' * (points[p][2] > 1)})")
               for p in levels]
    ax.legend(handles=handles, title="Penetration rate (smart controller share)", frameon=False,
              loc="upper left", bbox_to_anchor=(1.01, 1), alignment="left")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Fundamental diagram across runs, colored by penetration rate.")
    add_run_arguments(parser)
    parser.add_argument("--match", help="only runs whose name contains this text (e.g. a bag name)")
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    parser.add_argument("--name", help="output file name (default: built from --match and the box size)")
    args = parser.parse_args()

    run_dirs = resolve_runs(args.runs, all_runs=args.all or bool(args.match and not args.runs))
    if args.match:
        run_dirs = [d for d in run_dirs if args.match in d.name]
    if not run_dirs:
        parser.error("no runs selected")

    points = {}   # penetration rate -> (density, flow, run count)
    dT_used = args.dT
    for run_dir in run_dirs:
        p = penetration_rate(run_dir.name)
        boxes = density_and_flow_for(run_dir, args.dx, args.dT)
        dT_used = boxes["dT"]
        print(f"[{run_dir.name}] p={p:g}: {len(boxes['flow'])} boxes")
        density, flow, count = points.get(p, (np.empty(0), np.empty(0), 0))
        points[p] = (np.concatenate([density, boxes["density"]]),
                     np.concatenate([flow, boxes["flow"]]), count + 1)

    if len(run_dirs) == 1:
        name = run_dirs[0].name
    else:
        name = args.name or f"{args.match or 'runs'}_{len(run_dirs)}runs_dx{args.dx:g}_dT{dT_used:g}"
    title = f"Fundamental diagram: {args.match or name} ({len(run_dirs)} run{'s' * (len(run_dirs) > 1)})"

    out_dir = ROOT / "plots" / "fundamental_diagram"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.png"
    fig = plot_fundamental_diagram(points, args.dx, dT_used, title)
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
    print(f"Fundamental diagram: {path}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
