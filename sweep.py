"""
Run a full comparison sweep, then draw its figures and summary tables:

  1. main.py once per --smart controller, on every bag, at every penetration
     rate (in parallel), plus the shared p = 0 all-IDM baseline once per bag;
  2. per bag: plot_work_sweep.py, plot_flow_sweep.py and compare_flow.py
     (into plots/), named <bag>_seed<seed>_<controllers>;
  3. summarize.py: one table per bag (baseline, every run, crashes marked),
     copied to results/summaries/ in the repo when --results-dir is elsewhere.

    uv run sweep.py --results-dir D:/sweeps/wtj --smart WTJ0 WTJ1 WTJ2 WTJ3 WTJ4 TG
    uv run sweep.py --results-dir D:/sweeps/wtj --smart WTJ3 TG --bag wtj_real_drive --wtj3 gmin=15
    uv run sweep.py --results-dir D:/sweeps/wtj --smart WTJ3 TG --plots-only   # redraw from saved runs
    uv run sweep.py --results-dir D:/sweeps/wtj --smart WTJ4 TG --plots-only --series WTJ4 TG   # defaults only

Every run uses the same --seed (default 7), so each slot gets the same kind of
car and mass across controllers. Any option this script doesn't know (e.g.
--wtj3 gmin=15, --n-vehicles 50, --idm s0=3) is passed on to main.py.
"""

import argparse
import csv
import shutil
import subprocess
import sys
from pathlib import Path

from series import series_label
from summarize import run_key

ROOT = Path(__file__).parent
BAGS_DIR = ROOT / "bags"


def run(cmd):
    print("\n$ " + " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([sys.executable, *map(str, cmd)], cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description="Sweep controllers x bags x penetration rates, then plot and summarize.",
                                     epilog="Other options are passed on to main.py.")
    parser.add_argument("--results-dir", required=True,
                        help="where the runs go (pick a folder outside OneDrive: ~100-200 MB per run)")
    parser.add_argument("--smart", nargs="+", required=True, metavar="CTRL",
                        help="smart controllers to compare, e.g. WTJ0 WTJ1 WTJ2 WTJ3 WTJ4 TG")
    parser.add_argument("--bag", nargs="+", help="bag names in bags/ (default: every bag)")
    parser.add_argument("--penetration-rate", nargs="+", type=float, default=[0.25, 0.5, 0.75, 1.0], metavar="P",
                        help="rates above 0 (default 0.25 0.5 0.75 1); the p = 0 baseline is added once")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--jobs", type=int, default=16, help="simulations at once (default 16)")
    parser.add_argument("--plots-only", action="store_true", help="skip the simulations; plot and summarize saved runs")
    parser.add_argument("--series", nargs="+", metavar="LABEL",
                        help="only these series labels in the plots and tables, e.g. WTJ4 'WTJ4 beta=0.5' TG "
                             "(default: every saved variant of the --smart controllers)")
    args, passthrough = parser.parse_known_args()

    results = Path(args.results_dir).resolve()
    bags = args.bag or sorted(p.stem for p in BAGS_DIR.glob("*.bag"))
    bag_paths = [str(BAGS_DIR / f"{b}.bag") for b in bags]
    rates = [p for p in args.penetration_rate if p > 0]

    if not args.plots_only:
        for i, smart in enumerate(args.smart):
            run(["main.py", "--bag", *bag_paths, "--smart", smart, "--seed", args.seed, "--jobs", args.jobs,
                 "--results-dir", results, "--penetration-rate", *([0.0] if i == 0 else []), *rates, *passthrough])

    # Runs of this sweep, from runs.csv: its bags and seed, these controllers, and the
    # baseline; for repeated runs, the newest (summarize.run_key)
    with open(results / "runs.csv", newline="") as f:
        rows = [r for r in csv.DictReader(f)
                if r.get("source") in bags and r.get("seed") == str(args.seed)
                and (float(r["penetration_rate"]) == 0 or r.get("smart_controller") in args.smart)]
    latest = {run_key(r): r for r in rows}
    tag = f"seed{args.seed}_{'-'.join(args.smart)}"
    for bag in bags:
        mine = [r for r in latest.values() if r["source"] == bag
                and (not args.series or float(r["penetration_rate"]) == 0 or series_label(r) in args.series)]
        # Series labels, e.g. "WTJ3" or "WTJ3 gmin=15" when parameters were changed
        labels = list(dict.fromkeys(series_label(r) for r in sorted(
            mine, key=lambda r: args.smart.index(r["smart_controller"]) if r["smart_controller"] in args.smart else -1)
            if float(r["penetration_rate"]) > 0))
        runs = [results / "runs" / r["run"] for r in mine if (results / "runs" / r["run"] / "timeseries.csv").is_file()]
        if not labels or not runs:
            print(f"No runs for {bag}; skipping its plots")
            continue
        name = f"{bag}_{tag}"
        run(["plot_work_sweep.py", "--results", results / "runs.csv", "--source", bag, "--seed", args.seed,
             "--series", *labels, "--name", name])
        if any(float(r["penetration_rate"]) > 0 for r in mine if not r.get("collision")):
            run(["plot_flow_sweep.py", *runs, "--series", *labels, "--name", name])
            run(["compare_flow.py", *runs, "--series", *labels, "--name", name])
    run(["summarize.py", "--results", results, "--seed", args.seed, "--source", *bags,
         "--smart", *(args.series or args.smart), *(["--exact"] if args.series else []),
         "--name", "-".join(args.smart)])

    # The summary tables are small: keep a copy in the repo even when the runs live elsewhere
    keep = ROOT / "results" / "summaries"
    if results != keep.parent.resolve():
        keep.mkdir(parents=True, exist_ok=True)
        copies = [*(results / "summaries").glob(f"summary_*_{tag}.csv"), *results.glob(f"flow_density_stats_*_{tag}.csv")]
        for path in copies:
            shutil.copy2(path, keep / path.name)
        print(f"Copied {len(copies)} summary CSVs to {keep}")


if __name__ == "__main__":
    main()
