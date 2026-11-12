"""Compare the Python controllers in controllers.py against MATLAB outputs.

First run export_matlab_controllers.m in MATLAB. It writes a CSV with the
inputs and the MATLAB outputs for every test row. This script runs the Python
timeGap and wtj on the same rows and reports every row where they disagree.

Usage:
    python3 compare_controllers.py matlab_controller_outputs.csv
    python3 compare_controllers.py matlab_controller_outputs.csv --atol 1e-9 --show 20
    python3 compare_controllers.py matlab_controller_outputs.csv --save-mismatches bad_rows.csv

Exit code is 0 if everything matches, 1 if any row disagrees.
"""

import argparse
import math
import os
import sys

import numpy as np

# controllers.py lives one folder up, at the repo root.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import controllers  # noqa: E402

# controllers.py uses sqrt() without importing it. Supply it here so this
# script works without editing controllers.py; it does nothing if the import
# is added there later.
if not hasattr(controllers, "sqrt"):
    print("NOTE: controllers.py does not import sqrt; using math.sqrt for this comparison.\n")
    controllers.sqrt = math.sqrt


# Output limits of the Saturation blocks in the Simulink model (wtj.slx).
SIM_ACC_LOWER, SIM_ACC_UPPER = -3.0, 1.5


def timegap_branch(r):
    """The Simulink time-gap block uses lambda*min(0, sndot), so split rows by the sign of sndot."""
    return "sndot>0" if r["sndot"] > 0 else "sndot<=0"


def wtj_branch(r):
    """Return which branch of wtj a row goes through (helps find the cause of a mismatch)."""
    al = r["alpha"] * (r["sn"] - r["tau_max"] * r["vn"] - r["gmin"]) + r["lam"] * min(0, r["sndot"])
    au = r["alpha"] * (r["sn"] - r["tau_min"] * r["vn"] - r["gmin"]) + r["lam"] * min(0, r["sndot"])
    if r["vn"] <= 0:
        name, opt = "vn<=0 (returns al)", al
    else:
        a_star = r["c"] * math.sqrt(1 / (r["vn"] + 0.25)) - r["c"] * math.sqrt(1 / (30 + 0.25))
        opt = min(max(al, a_star), au)
        if a_star < al:
            name = "clamped to al"
        elif a_star > au:
            name = "clamped to au"
        else:
            name = "a_star"
    if opt < SIM_ACC_LOWER or opt > SIM_ACC_UPPER:
        name += f" + outside [{SIM_ACC_LOWER:g}, {SIM_ACC_UPPER:g}] (Simulink saturates)"
    return name


def run_python(rows):
    tg = np.array([
        controllers.timeGap(r["sn"], r["vn"], r["sndot"], r["alpha"], r["tau"], r["gmin"], r["lam"])
        for r in rows
    ], dtype=float)
    wt = np.array([
        controllers.wtj(r["sn"], r["vn"], r["sndot"], r["alpha"], r["tau_min"], r["tau_max"],
                        r["gmin"], r["lam"], r["c"])
        for r in rows
    ], dtype=float)
    return tg, wt


def compare(name, py, ml, rows, atol, rtol, show, branch_fn=None):
    both_nan = np.isnan(py) & np.isnan(ml)
    err = np.abs(py - ml)
    err[both_nan] = 0.0
    tol = atol + rtol * np.abs(ml)
    bad = np.where(~(err <= tol))[0]  # also catches NaN on only one side

    finite = err[np.isfinite(err)]
    print(f"=== {name} ===")
    print(f"  rows compared : {len(py)}")
    print(f"  max abs error : {finite.max() if finite.size else float('nan'):.3e}")
    print(f"  mean abs error: {finite.mean() if finite.size else float('nan'):.3e}")
    print(f"  mismatches    : {len(bad)}  (tolerance = {atol:g} + {rtol:g}*|matlab|)")

    if len(bad):
        edge = sum(rows[i]["case_id"] == 1 for i in bad)
        print(f"    {edge} edge-case rows, {len(bad) - edge} random rows")
        if branch_fn is not None:
            counts = {}
            for i in bad:
                b = branch_fn(rows[i])
                counts[b] = counts.get(b, 0) + 1
            print("    by branch: " + ", ".join(f"{k}: {v}" for k, v in sorted(counts.items())))
        worst = bad[np.argsort(-np.nan_to_num(err[bad], nan=np.inf))][:show]
        print(f"  worst {len(worst)}:")
        print(f"    {'row':>6} {'sn':>9} {'vn':>9} {'sndot':>9} {'python':>14} {'matlab':>14} {'abs err':>10}")
        for i in worst:
            r = rows[i]
            print(f"    {i:>6} {r['sn']:>9.4g} {r['vn']:>9.4g} {r['sndot']:>9.4g} "
                  f"{py[i]:>14.8g} {ml[i]:>14.8g} {err[i]:>10.3e}")
    print()
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV written by export_matlab_controllers.m")
    ap.add_argument("--atol", type=float, default=1e-9, help="absolute tolerance (default 1e-9)")
    ap.add_argument("--rtol", type=float, default=1e-9, help="relative tolerance (default 1e-9)")
    ap.add_argument("--show", type=int, default=10, help="how many of the worst mismatches to print")
    ap.add_argument("--save-mismatches", metavar="FILE", help="write mismatching rows to this CSV")
    args = ap.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    names = data.dtype.names
    required = {"case_id", "sn", "vn", "sndot", "alpha", "tau", "tau_min", "tau_max",
                "gmin", "lam", "c", "timeGap_matlab", "wtj_matlab"}
    missing = required - set(names)
    if missing:
        sys.exit(f"CSV is missing columns: {sorted(missing)}")

    rows = [{k: float(d[k]) for k in names} for d in np.atleast_1d(data)]
    tg_py, wtj_py = run_python(rows)

    bad_tg = compare("timeGap", tg_py, data["timeGap_matlab"], rows, args.atol, args.rtol, args.show,
                     branch_fn=timegap_branch)
    bad_wtj = compare("wtj", wtj_py, data["wtj_matlab"], rows, args.atol, args.rtol, args.show,
                      branch_fn=wtj_branch)

    if args.save_mismatches:
        idx = sorted(set(bad_tg) | set(bad_wtj))
        out_names = list(names) + ["timeGap_python", "wtj_python"]
        with open(args.save_mismatches, "w") as f:
            f.write(",".join(out_names) + "\n")
            for i in idx:
                vals = [rows[i][k] for k in names] + [tg_py[i], wtj_py[i]]
                f.write(",".join(f"{v:.17g}" for v in vals) + "\n")
        print(f"Wrote {len(idx)} mismatching rows to {args.save_mismatches}")

    ok = len(bad_tg) == 0 and len(bad_wtj) == 0
    print("RESULT:", "PASS - Python and MATLAB controllers match" if ok else "FAIL - see mismatches above")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
