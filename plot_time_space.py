"""
Time-space diagram of a run: every real vehicle's trajectory colored by speed,
with the boxes used for density and flow (density_flow.py) outlined.

    uv run plot_time_space.py                       # most recent run
    uv run plot_time_space.py RUN [RUN ...] --dx 500 --dT 60
    uv run plot_time_space.py --all

Plots go to plots/time_space/<run>.png (or variable_mass_results/plots/...).
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.patches import Rectangle

from density_flow import DEFAULT_DT, DEFAULT_DX, density_and_flow_for
from run_io import add_run_arguments, load_run, resolve_runs

CMAP = "RdYlGn"          # red = slow, green = fast, as in the lecture's diagrams
MAX_POINTS_PER_LINE = 3000


def plot_time_space(run, boxes):
    rows = run.rows(ghost=False)
    stride = max(1, len(run.time) // MAX_POINTS_PER_LINE)
    t = run.time[::stride]
    norm = plt.Normalize(0, np.percentile(run.velocity[rows], 99.5))

    fig, ax = plt.subplots(figsize=(12, 7))
    for k in rows:
        points = np.column_stack([t, run.position[k, ::stride]])
        lc = LineCollection(np.stack([points[:-1], points[1:]], axis=1), cmap=CMAP, norm=norm, linewidth=0.6)
        lc.set_array(run.velocity[k, ::stride][:-1])
        ax.add_collection(lc)
    ax.add_collection(PatchCollection(
        [Rectangle((t0, x0), boxes["dT"], boxes["dx"]) for t0, x0 in zip(boxes["t0"], boxes["x0"])],
        facecolor="none", edgecolor="black", linewidth=0.3, alpha=0.4))
    ax.autoscale()
    fig.colorbar(lc, ax=ax, label="Speed (m/s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title(f"{run.name}{run.collision_note()}\nTime-space diagram ({len(boxes['flow'])} boxes of "
                 f"{boxes['dx']:g} m x {boxes['dT']:g} s outlined)")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Time-space diagrams for runs.")
    add_run_arguments(parser)
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    args = parser.parse_args()

    for run_dir in resolve_runs(args.runs, all_runs=args.all):
        run = load_run(run_dir)
        boxes = density_and_flow_for(run_dir, args.dx, args.dT)
        out_dir = run.plots_dir / "time_space"
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{run.name}.png"
        plot_time_space(run, boxes).savefig(path, dpi=150, bbox_inches="tight")
        print(f"[{run.name}] {path}")
        if args.show:
            plt.show()
        plt.close("all")


if __name__ == "__main__":
    main()
