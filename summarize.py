"""
Summary table of a sweep: one row per run with its work, flow and safety, one
CSV per bag (source) plus one with every bag.

Every table has:
  - the baseline: the p = 0 run, where every follower runs the base controller
    (IDM), labelled "IDM (baseline)";
  - every smart-controller run, finished or not;
  - runs stopped by a collision, with status "collision" and the crash in the
    `collision` column, and no work or flow values.

Columns:
  status                   ok / inside 5 m (a smart car got closer than gmin) / collision
  avg_positive_work        m^2/s^3, averaged over followers (from runs.csv)
  flow_mean/median/p5/p95  veh/hr over the dx-by-dT boxes inside the platoon (density_flow.py)
  density_mean             veh/km over the same boxes
  throughput               veh/hr past marker_x_m: (cars past - 1) / (first-to-last time)
  cars_reached_marker      cars that got past marker_x_m (of n_vehicles)
  min_gap_smart_m          closest any smart car got to the car ahead
  min_gap_m, min_gap_type  closest any follower got, and its controller

    uv run summarize.py                                   # results/runs.csv, every bag
    uv run summarize.py --results D:/sweeps/results --seed 7
    uv run summarize.py --source wtj_real_drive --smart WTJ3 WTJ4 TG --name wtj3_vs_tg

The marker for throughput is the bag's entry in run_io.MARKER_X unless
--marker-x is given. Box flow and throughput are cached in each run folder, so
a second summary is fast. Tables go to <results>/summaries/.
"""

import argparse
import csv
from pathlib import Path

import numpy as np

from controllers import COMPARISON_CONTROLLERS, SMART_CONTROLLERS
from density_flow import DEFAULT_DT, DEFAULT_DX, density_and_flow_for, throughput
from run_io import marker_x_for

ROOT = Path(__file__).parent


def series_label(row):
    """'WTJ3' for default parameters, else the run's controller_params, e.g. 'WTJ3 gmin=5'."""
    params = row.get("controller_params") or "defaults"
    return row["smart_controller"] if params == "defaults" else params


def controller_of(label):
    return label.split(" ", 1)[0]


def is_unsafe(row):
    """True for a run that finished but had a smart car inside the safe gap (collisions are handled apart)."""
    return bool(row and row.get("unsafe")) and not row.get("collision")

COLUMNS = ["source", "role", "controller", "penetration_rate", "status", "avg_positive_work",
           "flow_mean_veh_per_hr", "flow_median_veh_per_hr", "flow_p5_veh_per_hr", "flow_p95_veh_per_hr",
           "density_mean_veh_per_km", "throughput_veh_per_hr", "marker_x_m", "cars_reached_marker",
           "min_gap_smart_m", "min_gap_m", "min_gap_type", "unsafe", "collision",
           "controller_params", "smart_definition", "base_controller", "n_vehicles", "seed", "run"]

CONTROLLER_ORDER = list(SMART_CONTROLLERS) + list(COMPARISON_CONTROLLERS)


def read_rows(results):
    """runs.csv rows, from the CSV itself or the results folder holding it."""
    path = Path(results)
    if path.is_dir():
        path = path / "runs.csv"
    if not path.is_file():
        raise SystemExit(f"No runs.csv at {path}")
    with open(path, newline="") as f:
        return path.parent, list(csv.DictReader(f))


def baseline_params(row):
    """The base controller's overrides in controller_params ('' when default)."""
    base = row.get("base_controller") or ""
    parts = [p.strip() for p in (row.get("controller_params") or "").split(";")]
    return "; ".join(p for p in parts if p.startswith(base + " "))


def run_key(row):
    """Runs with the same key are repeats of one another (same seed): only the newest is kept."""
    p = float(row["penetration_rate"])
    what = ("baseline", baseline_params(row)) if p == 0 else (series_label(row), "")
    return (row.get("source"), row.get("seed"), row.get("n_vehicles"), row.get("base_controller"), p, what)


def select(rows, args):
    out = []
    for r in rows:
        if not r.get("smart_controller"):
            continue                      # rows from before the smart_controller column
        if args.source and r.get("source") not in args.source:
            continue
        if args.seed is not None and r.get("seed") != str(args.seed):
            continue
        if args.base and r.get("base_controller") != args.base:
            continue
        if args.n_vehicles and r.get("n_vehicles") != str(args.n_vehicles):
            continue
        p = float(r["penetration_rate"])
        if p > 0 and args.smart and series_label(r) not in args.smart and (
                args.exact or r["smart_controller"] not in args.smart):
            continue                      # the baseline (p = 0) is always kept
        out.append(r)
    # Keep the newest of repeated runs (runs.csv is in time order); runs without a seed are all distinct
    latest = {}
    for i, r in enumerate(out):
        latest[run_key(r) if r.get("seed") else ("unseeded", i)] = r
    return list(latest.values())


def summarize_run(row, results_dir, dx, dT, marker_x):
    p = float(row["penetration_rate"])
    base = row.get("base_controller") or "base"
    rec = {k: "" for k in COLUMNS}
    rec.update({
        "source": row.get("source"), "penetration_rate": p,
        "role": "baseline" if p == 0 else "smart",
        "controller": f"{base} (baseline)" if p == 0 else series_label(row),
        "unsafe": row.get("unsafe", ""), "collision": row.get("collision", ""),
        "min_gap_smart_m": row.get("min_gap_smart_m", ""), "min_gap_m": row.get("min_gap_m", ""),
        "min_gap_type": row.get("min_gap_type", ""),
        "controller_params": row.get("controller_params", ""),
        "smart_definition": "" if p == 0 else row.get("smart_definition", ""),
        "base_controller": base, "n_vehicles": row.get("n_vehicles"), "seed": row.get("seed"),
        "run": row["run"],
    })
    if row.get("collision"):
        rec["status"] = "collision"
        return rec
    rec["status"] = "inside 5 m" if is_unsafe(row) else "ok"
    rec["avg_positive_work"] = round(float(row["avg_positive_work"]), 4)
    run_dir = results_dir / "runs" / row["run"]
    if not (run_dir / "timeseries.csv").is_file():
        rec["status"] += " (no run folder: flow not computed)"
        return rec
    boxes = density_and_flow_for(run_dir, dx, dT)
    flow, density = boxes["flow"], boxes["density"]
    if len(flow):
        rec.update({"flow_mean_veh_per_hr": round(float(np.mean(flow)), 1),
                    "flow_median_veh_per_hr": round(float(np.median(flow)), 1),
                    "flow_p5_veh_per_hr": round(float(np.percentile(flow, 5)), 1),
                    "flow_p95_veh_per_hr": round(float(np.percentile(flow, 95)), 1),
                    "density_mean_veh_per_km": round(float(np.mean(density)), 2)})
    rate, reached, cars = throughput(run_dir, marker_x)
    rec.update({"throughput_veh_per_hr": "" if np.isnan(rate) else round(float(rate), 1),
                "marker_x_m": marker_x, "cars_reached_marker": f"{reached}/{cars}"})
    return rec


def order(rec):
    """Baseline first, then controllers in CONTROLLER_ORDER (defaults before variants), then p."""
    ctrl = controller_of(rec["controller"])
    rank = CONTROLLER_ORDER.index(ctrl) if ctrl in CONTROLLER_ORDER else len(CONTROLLER_ORDER)
    return (rec["role"] != "baseline", rank, rec["controller"] != ctrl, rec["controller"], rec["penetration_rate"])


def write(path, recs):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(recs)


def print_table(source, recs):
    print(f"\n{source}  (throughput marker {next((r['marker_x_m'] for r in recs if r['marker_x_m'] != ''), '-')} m)")
    print(f"  {'controller':22} {'p':>5} {'work':>7} {'avg flow':>9} {'med flow':>9} {'thru':>6}  status")
    for r in recs:
        def f(k, fmt):
            return "" if r[k] == "" else fmt.format(r[k])
        status = r["status"] + (f" ({r['min_gap_smart_m']} m)" if r["status"].startswith("inside") else "")
        print(f"  {r['controller']:22} {r['penetration_rate']:>5g} {f('avg_positive_work', '{:.3f}'):>7} "
              f"{f('flow_mean_veh_per_hr', '{:.0f}'):>9} {f('flow_median_veh_per_hr', '{:.0f}'):>9} "
              f"{f('throughput_veh_per_hr', '{:.0f}'):>6}  {status}")


def main():
    parser = argparse.ArgumentParser(description="Per-bag summary tables of work, flow and safety for a sweep.")
    parser.add_argument("--results", default=str(ROOT / "results"),
                        help="results folder (or its runs.csv); default results/")
    parser.add_argument("--source", nargs="+", help="only these bags (or 'sine'); default every source")
    parser.add_argument("--seed", type=int, help="only runs made with this --seed")
    parser.add_argument("--base", help="only runs with this base controller")
    parser.add_argument("--n-vehicles", type=int, help="only runs with this many followers")
    parser.add_argument("--smart", nargs="+", metavar="LABEL",
                        help="only these smart controllers or series labels (e.g. WTJ3 'WTJ3 gmin=15' TG); "
                             "the baseline is always included")
    parser.add_argument("--exact", action="store_true",
                        help="--smart names match series labels only: WTJ4 is the default WTJ4, "
                             "not 'WTJ4 beta=0.5' too")
    parser.add_argument("--dx", type=float, default=DEFAULT_DX, help=f"box length in m (default {DEFAULT_DX:g})")
    parser.add_argument("--dT", type=float, default=DEFAULT_DT, help=f"box duration in s (default {DEFAULT_DT:g})")
    parser.add_argument("--marker-x", type=float, help="throughput marker in m for every bag "
                                                        "(default: run_io.MARKER_X per bag)")
    parser.add_argument("--name", help="added to the output file names")
    parser.add_argument("--quiet", action="store_true", help="don't print the tables")
    args = parser.parse_args()

    results_dir, rows = read_rows(args.results)
    selected = select(rows, args)
    if not selected:
        parser.error("no runs match")

    recs = []
    for r in selected:
        marker = args.marker_x if args.marker_x is not None else marker_x_for(r.get("source"))
        recs.append(summarize_run(r, results_dir, args.dx, args.dT, marker))
    recs.sort(key=lambda r: (r["source"], *order(r)))

    suffix = "".join(f"_{s}" for s in (f"seed{args.seed}" if args.seed is not None else None, args.name) if s)
    out_dir = results_dir / "summaries"
    sources = sorted({r["source"] for r in recs})
    for source in sources:
        mine = [r for r in recs if r["source"] == source]
        if not any(r["role"] == "baseline" for r in mine):
            print(f"Note: no p = 0 baseline run for {source} in {results_dir / 'runs.csv'}")
        write(out_dir / f"summary_{source}{suffix}.csv", mine)
        if not args.quiet:
            print_table(source, mine)
    write(out_dir / f"summary_all{suffix}.csv", recs)
    n_crash = sum(r["status"] == "collision" for r in recs)
    n_close = sum(r["status"].startswith("inside") for r in recs)
    print(f"\n{len(recs)} runs ({n_crash} collisions, {n_close} inside 5 m) in {len(sources)} bag(s). "
          f"Tables: {out_dir}{'/summary_<bag>' + suffix + '.csv'} and summary_all{suffix}.csv")


if __name__ == "__main__":
    main()
