"""
Plot a run from its CSVs: positions, speeds, accelerations and positive work.

    uv run plot_run.py                      # most recent run
    uv run plot_run.py RUN_NAME [RUN ...]   # run names or folders
    uv run plot_run.py --all                # every run

Plots go to plots/<graph>/<run>.png, or variable_mass_results/plots/... for
runs saved there.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

from metrics import total_positive_work
from run_io import add_run_arguments, load_run, resolve_runs

# The leader (near-black) and IDM (gray) are neutral: the reference and the background traffic.
# Smart controllers use the same colors as series.py.
TYPE_COLORS = {"lead": "#0b0b0b", "WTJ0": "#2a78d6", "WTJ1": "#1baf7a", "WTJ2": "#6250d6", "WTJ3": "#008300",
               "WTJ4": "#eda100",
               "TG": "red", "STG": "darkred", "IDM": "#8a8985"}
BAR_ALPHA = 0.6     # Opacity of the real cars' work bars
GHOST_ALPHA = 0.2   # Opacity of the ghost work bars behind them


def color(vtype):
    return TYPE_COLORS.get(vtype, "gray")


def title_for(run):
    """Run name plus fixed/variable mass, e.g. '..._wtj_real_drive_n100_p0.5, fixed mass'."""
    return f"{run.name}, {'variable' if run.variable_mass else 'fixed'} mass{run.collision_note()}"


def plot_state(run, values, ylabel):
    """One line per real vehicle (ghosts left out), colored by controller."""
    fig, ax = plt.subplots(figsize=(10, 6))
    for k in run.rows(ghost=False):
        ax.plot(run.time, values[k], color=color(run.vehicles[k]["type"]), linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title_for(run))
    types = sorted({run.vehicles[k]["type"] for k in run.rows(ghost=False)}, key=list(TYPE_COLORS).index)
    ax.legend(handles=[Patch(color=color(t), label=t) for t in types])
    return fig


def plot_work(run):
    """
    Average positive work per follower, sum max(a v, 0) dt / T, colored by
    controller. Ghosts are fainter bars behind their WTJ car's bar.
    """
    duration = len(run.time) * run.dt
    work = total_positive_work(run.velocity, run.acceleration, run.dt) / duration
    followers = [k for k in run.rows(ghost=False) if run.vehicles[k]["vehicle"] > 0]
    ghosts = run.rows(ghost=True)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar([run.vehicles[k]["vehicle"] for k in followers], work[followers],
           color=[color(run.vehicles[k]["type"]) for k in followers], alpha=BAR_ALPHA, zorder=2)
    mean = work[followers].mean()
    mean_line = ax.axhline(mean, color="black", linestyle="--", linewidth=1, label=f"mean {mean:.4f}")

    types = sorted({run.vehicles[k]["type"] for k in followers}, key=list(TYPE_COLORS).index)
    handles = [Patch(color=color(t), alpha=BAR_ALPHA, label=t) for t in types] + [mean_line]
    for gtype in dict.fromkeys(run.vehicles[k]["type"] for k in ghosts):
        mine = [k for k in ghosts if run.vehicles[k]["type"] == gtype]
        ax.bar([run.vehicles[k]["vehicle"] for k in mine], work[mine],
               color=color(gtype), alpha=GHOST_ALPHA, zorder=1)
        handles.append(Patch(color=color(gtype), alpha=GHOST_ALPHA, label=f"{gtype} ghost"))

    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("Car (position behind leader)")
    ax.set_ylabel("Average positive work (m²/s³)")
    ax.set_title(title_for(run))
    ax.legend(handles=handles)
    return fig


def save(fig, run, graph):
    out_dir = run.plots_dir / graph
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{run.name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    return path


def main():
    parser = argparse.ArgumentParser(description="Plot positions, speeds, accelerations and positive work for runs.")
    add_run_arguments(parser)
    args = parser.parse_args()

    for run_dir in resolve_runs(args.runs, all_runs=args.all):
        run = load_run(run_dir)
        figures = {
            "positions": plot_state(run, run.position, "Position (m)"),
            "speeds": plot_state(run, run.velocity, "Speed (m/s)"),
            "accelerations": plot_state(run, run.acceleration, "Acceleration (m/s²)"),
            "positive_work": plot_work(run),
        }
        for graph, fig in figures.items():
            print(f"[{run.name}] {graph}: {save(fig, run, graph)}")
        if args.show:
            plt.show()
        plt.close("all")


if __name__ == "__main__":
    main()
