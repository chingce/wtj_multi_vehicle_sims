"""
Flow and density statistics for a set of runs, and one figure comparing their
flow distributions.

Each run's flow and density come from density_flow.py (one value per
dx-by-dT box inside the platoon). For every run this writes one row of
statistics (box count, mean, median, percentiles, max) to
flow_density_stats_<name>.csv, and draws box plots of the per-box flow,
grouped by penetration rate.

Panels (see series.py): the smart controllers at default parameters (WTJ,
WTJ3, WTJ4, TG), then one panel per controller that also has runs with changed
parameters (e.g. "WTJ3 gmin=5"), with its default boxes repeated for
reference. A p = 0 run has no smart cars, so it is drawn in every series.
Runs stopped by a collision (in runs.csv, no run folder) are marked with an x.

    uv run compare_flow.py --all --match realRun_n100
    uv run compare_flow.py RUN [RUN ...] --dx 500 --dT 60 --name my_sweep

Each run's series comes from the runs.csv next to its runs/ folder
(smart_controller and controller_params columns), falling back to the run
name (_WTJ2, _WTJ3, _WTJ4, _TG). Outputs: <results>/flow_density_stats_<name>.csv and
plots/flow_distribution/<name>.png.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from density_flow import DEFAULT_DT, DEFAULT_DX, density_and_flow_for
from run_io import add_run_arguments, penetration_rate, resolve_runs
from series import is_secondary, panels, series_label

ROOT = Path(__file__).parent

SURFACE = "#fcfcfb"
GRID = "#e4e3df"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"


def runs_csv_rows(results_dir):
    path = results_dir / "runs.csv"
    if not path.is_file():
        return {}
    with open(path, newline="") as f:
        return {r["run"]: r for r in csv.DictReader(f)}


def label_for(run_dir, rows):
    """Series label of a run (series.series_label), falling back to the run name."""
    row = rows.get(run_dir.name)
    if row and row.get("smart_controller"):
        return series_label(row)
    for name in ("WTJ2", "WTJ3", "WTJ4", "TG"):
        if f"_{name}" in run_dir.name.split("_n", 1)[-1]:
            return name
    return "WTJ"


def stats(values):
    return {"mean": values.mean(), "median": np.median(values),
            "p5": np.percentile(values, 5), "p25": np.percentile(values, 25),
            "p75": np.percentile(values, 75), "p95": np.percentile(values, 95), "max": values.max()}


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY)


def plot_flow_distributions(flows, baseline, crashed, title, only=None):
    """
    flows: {label: {p: flow array}} for p > 0; baseline: flow array of the p = 0
    run (or None); crashed: {label: {p}} runs stopped by a collision.
    """
    layout = panels(set(flows) | set(crashed), only)
    levels = sorted({p for lab in flows for p in flows[lab]} | {p for lab in crashed for p in crashed[lab]}
                    | ({0.0} if baseline is not None else set()))
    fig, axes = plt.subplots(len(layout), 1, figsize=(13, 5.2 * len(layout)), sharey=True,
                             facecolor=SURFACE, squeeze=False)
    lowest = min([0.0] + [f.min() for lab in flows for f in flows[lab].values()]
                 + ([baseline.min()] if baseline is not None else []))

    for ax, (panel_title, series) in zip(axes[:, 0], layout):
        style_axes(ax)
        width = 0.8 / len(series)
        for j, (label, color, _marker) in enumerate(series):
            offset = (j - (len(series) - 1) / 2) * width
            for p in levels:
                x = levels.index(p) + offset
                if p == 0 and not flows.get(label):
                    continue   # every run of this series was stopped: no baseline box, only x marks
                flow = baseline if p == 0 else flows.get(label, {}).get(p)
                if flow is not None:
                    ax.boxplot([flow], positions=[x], widths=width * 0.8, patch_artist=True, whis=(5, 95),
                               flierprops=dict(marker="o", markersize=2.5, markerfacecolor=color,
                                               markeredgecolor="none", alpha=0.5),
                               medianprops=dict(color=SURFACE, linewidth=2),
                               whiskerprops=dict(color=color, linewidth=1.2),
                               capprops=dict(color=color, linewidth=1.2),
                               boxprops=dict(facecolor=color, edgecolor=color, alpha=0.45 if p == 0 else 0.85,
                                             hatch="////" if is_secondary(label) else None))
                    ax.scatter([x], [flow.mean()], marker="D", s=20, color=TEXT, zorder=4)
                elif p in crashed.get(label, set()):
                    ax.scatter([x], [lowest], marker="x", s=80, color=color, linewidths=2.4, zorder=4, clip_on=False)

        ax.set_xticks(range(len(levels)))
        ax.set_xticklabels([f"p = {p:g}" + ("\n(all IDM, same run)" if p == 0 else "") for p in levels])
        ax.set_xlim(-0.6, len(levels) - 0.4)
        ax.set_ylabel("Flow per box (veh/hr)")
        ax.set_title(panel_title, loc="left", fontsize=11)
        handles = [Patch(facecolor=color, alpha=0.85, label=label, hatch="////" if is_secondary(label) else None)
                   for label, color, _ in series]
        handles.append(Line2D([], [], marker="D", linestyle="", color=TEXT, markersize=5, label="mean"))
        if any(crashed.get(label) for label, *_ in series):
            handles.append(Line2D([], [], marker="x", linestyle="", color=TEXT_SECONDARY, markersize=8,
                                  markeredgewidth=2, label="stopped by a collision"))
        ax.legend(handles=handles, frameon=False, loc="upper left", bbox_to_anchor=(1.01, 1))

    axes[0, 0].set_ylim(bottom=lowest)
    axes[-1, 0].set_xlabel("Penetration rate (share of cars running the smart controller)")
    axes[0, 0].text(1.01, 0.35, "Box: 25th-75th percentile\nLine: median\nWhiskers: 5th-95th\nDots: outside 5-95",
                    transform=axes[0, 0].transAxes, va="top", fontsize=9, color=TEXT_SECONDARY)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12)
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Flow/density statistics and flow distributions across runs.")
    add_run_arguments(parser)
    parser.add_argument("--match", help="only runs whose name contains this text (e.g. realRun_n100)")
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    parser.add_argument("--defaults-only", action="store_true",
                        help="only controllers at default parameters (no variants panel)")
    parser.add_argument("--series", nargs="+", metavar="LABEL",
                        help="only these series, in one panel, e.g. TG 'TG gmin=5' WTJ3 'WTJ3 gmin=5'. "
                             "A variant takes its controller's color, drawn dashed/hollow/hatched")
    parser.add_argument("--name", help="name for the outputs (default: built from --match and the box size)")
    args = parser.parse_args()

    run_dirs = resolve_runs(args.runs, all_runs=args.all or bool(args.match and not args.runs))
    if args.match:
        run_dirs = [d for d in run_dirs if args.match in d.name]
    if not run_dirs:
        parser.error("no runs selected")
    results_dir = run_dirs[0].parent.parent
    rows = runs_csv_rows(results_dir)

    stat_rows, flows, crashed, baseline = [], defaultdict(dict), defaultdict(set), None
    dT_used = args.dT
    if args.defaults_only:
        run_dirs = [d for d in run_dirs
                    if (rows.get(d.name, {}).get("controller_params") or "defaults") == "defaults"]
    if args.series:   # the chosen series, plus the shared p = 0 baseline
        run_dirs = [d for d in run_dirs if penetration_rate(d.name) == 0 or label_for(d, rows) in args.series]
    for run_dir in sorted(run_dirs, key=lambda d: (penetration_rate(d.name), d.name)):
        p = penetration_rate(run_dir.name)
        label = label_for(run_dir, rows)
        boxes = density_and_flow_for(run_dir, args.dx, args.dT)
        dT_used = boxes["dT"]
        flow, density = boxes["flow"], boxes["density"]
        row = {"run": run_dir.name, "penetration_rate": p,
               "series": "none (all base)" if p == 0 else label, "n_boxes": len(flow)}
        row.update({f"flow_{k}_veh_per_hr": round(float(v), 2) for k, v in stats(flow).items()})
        row.update({f"density_{k}_veh_per_km": round(float(v), 3) for k, v in stats(density).items()})
        stat_rows.append(row)
        print(f"[{run_dir.name}] p={p:g} {row['series']}: {len(flow)} boxes, flow median "
              f"{np.median(flow):.0f} / p95 {np.percentile(flow, 95):.0f} veh/hr, "
              f"density median {np.median(density):.1f} veh/km")
        if p == 0:
            baseline = flow
        else:
            flows[label][p] = flow

    # Runs in the same sweep that were stopped by a collision have a runs.csv row but no folder
    # (same seed as the selected runs, so they belong to the same sweep)
    names = {d.name for d in run_dirs}
    seeds = {rows[n].get("seed") for n in names if n in rows}
    for r in rows.values():
        if (r.get("collision") and r["run"] not in names and r.get("seed") in seeds
                and r.get("smart_controller") and float(r["penetration_rate"]) > 0
                and (not args.match or args.match in r["run"])):
            if args.defaults_only and (r.get("controller_params") or "defaults") != "defaults":
                continue
            if args.series and series_label(r) not in args.series:
                continue
            crashed[series_label(r)].add(float(r["penetration_rate"]))
            print(f"  {r['run']}: stopped by a collision, marked with x")

    name = args.name or f"{args.match or 'runs'}_dx{args.dx:g}_dT{dT_used:g}"
    stats_path = results_dir / f"flow_density_stats_{name}.csv"
    with open(stats_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(stat_rows[0]))
        writer.writeheader()
        writer.writerows(stat_rows)
    print(f"Statistics: {stats_path}")

    n_crashed = sum(len(v) for v in crashed.values())
    title = (f"Flow distribution per run: {args.match or name}, one value per {args.dx:g} m x {dT_used:g} s "
             f"box inside the platoon; {len(run_dirs)} runs" + (f", {n_crashed} stopped by a collision" if n_crashed else ""))
    out_dir = ROOT / "plots" / "flow_distribution"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.png"
    plot_flow_distributions(flows, baseline, crashed, title, args.series).savefig(path, dpi=150, bbox_inches="tight",
                                                                     facecolor=SURFACE)
    print(f"Flow distributions: {path}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
