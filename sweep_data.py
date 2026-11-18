"""
Shared plumbing for iteration.py and final_model.py: which runs a study needs,
finding them in runs.csv (running main.py for any that are missing), and their
summary metrics.

A study is a list of Configs. A Config is a controller preset plus parameter
overrides, e.g. Config("STG", {"tau": 3.0}). Each one is run on every bag at
every penetration rate (seed 7 by default), next to the all-IDM baseline
(p = 0) of each bag.

A run matches a Config when its `smart_definition` in runs.csv (the full
parameter set, e.g. "STG = SpecialTimeGapController(alpha=1, tau=3, gmin=5,
lam=2)") is the Config's, and it has the same bag, rate, seed, fleet size and
base controller. Of repeated runs the newest wins. Matching on the full
definition keeps runs made under an older default apart from new ones.

Results folder: --results-dir, else the WTJ_RESULTS_DIR environment variable,
else results/ in the repo. Full sweeps take ~75 GB, so keep it outside OneDrive.
"""

import csv
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from controllers import preset_label
from density_flow import DEFAULT_DT, DEFAULT_DX
from run_io import bag_label, marker_x_for, repair_csv
from summarize import baseline_params, summarize_run

ROOT = Path(__file__).parent
BAGS_DIR = ROOT / "bags"
RATES = (0.25, 0.5, 0.75, 1.0)
SEED = 7
N_VEHICLES = 100
BASE = "IDM"


def all_bags():
    """Bag names (file stems) in bags/, sorted."""
    return sorted(p.stem for p in BAGS_DIR.glob("*.bag"))


def results_dir(arg=None):
    return Path(arg or os.environ.get("WTJ_RESULTS_DIR") or ROOT / "results").resolve()


@dataclass(frozen=True)
class Config:
    """A smart-controller setting: preset name plus parameter overrides."""
    smart: str
    params: dict = field(default_factory=dict, hash=False, compare=False)
    label: str = ""           # display name; default built from the preset and overrides

    def __post_init__(self):
        if not self.label:
            extra = " ".join(f"{k}={v:g}" for k, v in self.params.items())
            object.__setattr__(self, "label", f"{self.smart} {extra}".strip())

    @property
    def definition(self):
        return preset_label(self.smart, self.params)

    def __hash__(self):
        return hash(self.definition)

    def __eq__(self, other):
        return isinstance(other, Config) and self.definition == other.definition


BASELINE = "IDM (baseline)"


def read_runs(results):
    path = results / "runs.csv"
    if not path.is_file():
        return []
    repair_csv(path)
    with open(path, newline="") as f:
        return [r for r in csv.DictReader(f) if r.get("smart_definition") is not None]


def _matches(row, bag, p, seed, n):
    return (row.get("source") == bag and row.get("seed") == str(seed) and row.get("n_vehicles") == str(n)
            and row.get("base_controller") == BASE and float(row["penetration_rate"]) == p)


def find_run(rows, cfg, bag, p, seed=SEED, n=N_VEHICLES):
    """Newest runs.csv row for cfg (or the baseline when cfg is None and p == 0), else None."""
    hit = None
    for r in rows:
        if not _matches(r, bag, p, seed, n):
            continue
        if cfg is None:
            if baseline_params(r) == "":
                hit = r
        elif r.get("smart_definition") == cfg.definition:
            hit = r
    return hit


def ensure_runs(results, configs, bags, rates=RATES, seed=SEED, n=N_VEHICLES, jobs=8, dry_run=False):
    """
    Run main.py for every (config, bag, rate) and baseline without a matching
    row in runs.csv. Returns the number of simulations started.
    """
    rows = read_runs(results)
    todo = []   # (cfg or None, bag, [rates])
    for bag in bags:
        if find_run(rows, None, bag, 0.0, seed, n) is None:
            todo.append((None, bag, [0.0]))
    for cfg in configs:
        for bag in bags:
            missing = [p for p in rates if find_run(rows, cfg, bag, p, seed, n) is None]
            if missing:
                todo.append((cfg, bag, missing))
    # One main.py call per (config, set of missing rates), over every bag that needs it
    groups = {}
    for cfg, bag, ps in todo:
        groups.setdefault((cfg, tuple(ps)), []).append(bag)
    started = 0
    for (cfg, ps), group_bags in groups.items():
        smart = cfg.smart if cfg else "TG"
        cmd = [sys.executable, "main.py", "--bag", *[str(BAGS_DIR / f"{b}.bag") for b in group_bags],
               "--smart", smart, "--seed", str(seed), "--n-vehicles", str(n), "--jobs", str(jobs),
               "--base", BASE, "--results-dir", str(results), "--penetration-rate", *[f"{p:g}" for p in ps]]
        if cfg and cfg.params:
            cmd += [f"--{smart.lower()}", *[f"{k}={v:g}" for k, v in cfg.params.items()]]
        what = cfg.label if cfg else BASELINE
        print(f"  run {what}: {len(group_bags)} bag(s) x p = {', '.join(f'{p:g}' for p in ps)}", flush=True)
        started += len(group_bags) * len(ps)
        if not dry_run:
            subprocess.run(cmd, cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    return started


class Study:
    """Rows and metrics for a set of configs on a set of bags, plus each bag's baseline."""

    def __init__(self, results, configs, bags, rates=RATES, seed=SEED, n=N_VEHICLES,
                 dx=DEFAULT_DX, dT=DEFAULT_DT):
        self.results, self.configs, self.bags, self.rates = results, list(configs), list(bags), list(rates)
        self.dx, self.dT = dx, dT
        rows = read_runs(results)
        self.rows = {}           # (cfg | None, bag, p) -> runs.csv row
        for bag in self.bags:
            self.rows[(None, bag, 0.0)] = find_run(rows, None, bag, 0.0, seed, n)
            for cfg in self.configs:
                for p in self.rates:
                    self.rows[(cfg, bag, p)] = find_run(rows, cfg, bag, p, seed, n)
        self._metrics = {}

    def row(self, cfg, bag, p):
        return self.rows.get((cfg, bag, 0.0 if cfg is None else p))

    def metrics(self, cfg, bag, p):
        """summarize.summarize_run record (status, work, flow, throughput, gaps...), or None if not run."""
        key = (cfg, bag, 0.0 if cfg is None else p)
        if key not in self._metrics:
            r = self.rows.get(key)
            self._metrics[key] = None if r is None else summarize_run(r, self.results, self.dx, self.dT,
                                                                      marker_x_for(bag))
        return self._metrics[key]

    def run_dir(self, cfg, bag, p):
        r = self.row(cfg, bag, p)
        if r is None:
            return None
        d = self.results / "runs" / r["run"]
        return d if (d / "timeseries.csv").is_file() else None

    def change(self, cfg, p, col, ref=None, bags=None):
        """
        Mean over bags of value / reference - 1, in %. Reference: the bag's
        baseline, or config `ref` at the same p. None if any bag is missing or
        crashed (for cfg or ref).
        """
        vals = []
        for bag in bags or self.bags:
            m = self.metrics(cfg, bag, p)
            t = self.metrics(ref, bag, p)
            if m is None or t is None or m["status"] == "collision" or t["status"] == "collision":
                return None
            vals.append(float(m[col]) / float(t[col]) - 1)
        return 100 * float(np.mean(vals))

    def safety(self, cfg, rates=None):
        """(collisions, runs inside 5 m, runs, closest smart-car gap in m) over bags and rates."""
        recs = [self.metrics(cfg, b, p) for b in self.bags for p in (rates or self.rates)]
        recs = [m for m in recs if m is not None]
        gaps = [float(m["min_gap_smart_m"]) for m in recs if m["min_gap_smart_m"] not in ("", None)]
        return (sum(m["status"] == "collision" for m in recs), sum(m["status"].startswith("inside") for m in recs),
                len(recs), min(gaps) if gaps else float("nan"))

    def records(self):
        """Every metrics record, baseline first per bag (for CSV tables)."""
        out = []
        for bag in self.bags:
            m = self.metrics(None, bag, 0.0)
            if m:
                out.append({**m, "controller": BASELINE, "bag_label": bag_label(bag)})
            for cfg in self.configs:
                for p in self.rates:
                    m = self.metrics(cfg, bag, p)
                    if m:
                        out.append({**m, "controller": cfg.label, "bag_label": bag_label(bag)})
        return out


METRICS = [("avg_positive_work", "Work"), ("flow_mean_veh_per_hr", "Avg box flow"),
           ("throughput_veh_per_hr", "Throughput")]


EXPORT_CSV = False   # iteration.py / final_model.py set this from --csv


def write_csv(path, recs, fields=None):
    """Write a CSV table, only when CSV export is on (EXPORT_CSV)."""
    if not EXPORT_CSV:
        return None
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = fields or list(dict.fromkeys(k for r in recs for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(recs)
    return path
