"""
The controller iteration, from the original WTJ controller to the Special Time
Gap (STG) controller: runs every simulation it needs (only the missing ones),
then writes the iteration tables of the report.

    uv run iteration.py --results-dir C:/Users/me/wtj_results    # run what's missing, then the tables
    uv run iteration.py --results-dir ... --no-run                 # tables from saved runs only
    uv run iteration.py --results-dir ... --dry-run                # list what would be run

Stages (seed 7, 100 vehicles, IDM base, every bag, p = 0.25 0.5 0.75 1, plus
the all-IDM p = 0 baseline of each bag):

  1. Original WTJ:             WTJ0 (lam 0.5), WTJ1 (lam 2), with TG for reference
  2. Soft lower bound:         WTJ2 (beta 0), WTJ3 (beta 0.1), WTJ4 (beta 0.25)
  3. Beta sweep:               WTJSoftLowerBound, beta 0-1, tau_min 2 and 2.5
  4. Special time gap:         STG lam 0.1 and lam 2, against TG, WTJ4 and beta = 1
  5. Time gap sweep:           TG and STG, tau = 2, 2.5, 3, 4
  6. Damping sweep:            STG (tau 2.5), lam = 1, 2, 3, 4

Outputs:
  report/tables/iteration_path.tex          every candidate, in the order tried
  report/tables/final_vs_alternatives.tex   STG vs TG, WTJ4 and beta = 1, per penetration rate
  report/tables/tuning.tex                  the tau and lam sweeps
  results/iteration/*.csv                   with --csv: one table per stage, plus every run
"""

import argparse
from pathlib import Path

import numpy as np

import sweep_data
from sweep_data import METRICS, RATES, Config, Study, all_bags, ensure_runs, results_dir, write_csv

ROOT = Path(__file__).parent
TABLES = ROOT / "results" / "iteration"
TEX = ROOT / "report" / "tables"

# --- the configurations of every stage -------------------------------------------------------
TG = Config("TG", {"tau": 2.5}, "TG")
WTJ0, WTJ1 = Config("WTJ0"), Config("WTJ1")
WTJ2, WTJ3, WTJ4 = Config("WTJ2"), Config("WTJ3"), Config("WTJ4")
STAGE12 = [WTJ0, WTJ1, WTJ2, WTJ3, WTJ4, TG]

BETAS = [0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
PRESET_BETA = {0.0: WTJ2, 0.1: WTJ3, 0.25: WTJ4}


def beta_config(beta, tau_min):
    if tau_min == 2.0 and beta in PRESET_BETA:
        return PRESET_BETA[beta]
    params = {"beta": beta} if tau_min == 2.0 else {"beta": beta, "tau_min": tau_min}
    return Config("WTJ4", params, f"WTJSoftLB beta={beta:g} tau_min={tau_min:g}")


TAU_MINS = [2.0, 2.5]
BETA_GRID = {(t, b): beta_config(b, t) for t in TAU_MINS for b in BETAS}

STG_LAM01 = Config("STG", {"lam": 0.1}, "STG lam=0.1")
STG = Config("STG", {"tau": 2.5, "lam": 2.0}, "STG")
BETA1 = BETA_GRID[(2.0, 1.0)]
STAGE4 = [TG, STG_LAM01, STG, WTJ4, BETA1]

TAUS = [2.0, 2.5, 3.0, 4.0]
TAU_GRID = {(c, t): Config(c, {"tau": t}, f"{c} tau={t:g}") for c in ("TG", "STG") for t in TAUS}
LAMS = [1.0, 2.0, 3.0, 4.0]
LAM_GRID = {l: Config("STG", {"lam": l}, f"STG lam={l:g}") for l in LAMS}

ALL_CONFIGS = list(dict.fromkeys(STAGE12 + list(BETA_GRID.values()) + STAGE4 + list(TAU_GRID.values())
                                 + list(LAM_GRID.values())))


# --- LaTeX tables for report/main.tex -------------------------------------------------------
def fmt(v, digits=1, signed=True):
    if v is None or v == "" or (isinstance(v, float) and np.isnan(v)):
        return "--"
    if not signed:
        return f"{v:.{digits}f}"
    s = f"{v:+.{digits}f}"
    return f"${s[0]}${s[1:]}"   # LaTeX minus/plus sign, digits stay in text (so \textbf applies)


def summary_rows(study, configs, names=None):
    """Per config: safety and mean change vs IDM (over bags, per p and over p)."""
    out = []
    for cfg in configs:
        crash, inside, n, gap = study.safety(cfg)
        rec = {"controller": (names or {}).get(cfg, cfg.label), "definition": cfg.definition,
               "collisions": f"{crash}/{n}", "inside_5m": f"{inside}/{n}", "min_gap_smart_m": round(gap, 2)}
        for col, lab in METRICS:
            per = [study.change(cfg, p, col) for p in study.rates]
            for p, v in zip(study.rates, per):
                rec[f"{col}_p{p:g}_pct"] = "" if v is None else round(v, 2)
            rec[f"{col}_mean_pct"] = "" if any(v is None for v in per) else round(float(np.mean(per)), 2)
        out.append(rec)
    return out


def mean_change(study, cfg, col, ref=None, rates=RATES):
    vals = [study.change(cfg, p, col, ref=ref) for p in rates]
    return None if any(v is None for v in vals) else float(np.mean(vals))


def safety_cells(study, cfg):
    crash, inside, n, gap = study.safety(cfg)
    return [f"{crash}/{n}", f"{inside}/{n}", "--" if crash == n else f"{gap:.2f}"]


def bold(row):
    return [rf"\textbf{{{c}}}" for c in row]


def tex_iteration_path(study):
    """One row per candidate controller, in the order they were tried."""
    W, F = "avg_positive_work", "flow_mean_veh_per_hr"
    path = [
        ("1", "WTJ0", r"original WTJ, $\lambda=0.5$", WTJ0),
        ("1", "WTJ1", r"stronger damping, $\lambda=2$", WTJ1),
        ("2", "WTJ2", r"soft lower bound, $\beta=0$, $\lambda=3$", WTJ2),
        ("2", "WTJ3", r"$\beta=0.1$", WTJ3),
        ("2", "WTJ4", r"$\beta=0.25$", WTJ4),
        ("3", "WTJSoftLB", r"$\beta=1$ (best of the $\beta$ sweep)", BETA1),
        ("--", "TG", r"time gap, two-sided $\lambda=0.1$ (reference)", TG),
        ("4", "STG", r"TG with braking-only damping, $\lambda=0.1$", STG_LAM01),
        ("4--6", "STG (final)", r"braking-only $\lambda=2$, $\tau=2.5$\,s", STG),
    ]
    rows = []
    for stage, name, change, cfg in path:
        vs_tg = (["ref.", "ref."] if cfg is TG else
                 [fmt(mean_change(study, cfg, W, ref=TG)), fmt(mean_change(study, cfg, F, ref=TG))])
        row = [stage, name, change, *safety_cells(study, cfg), fmt(mean_change(study, cfg, W)),
               fmt(mean_change(study, cfg, F)), *vs_tg]
        rows.append(bold(row) if cfg is STG else row)
    header = ["Stage", "Controller", "What changed", "Coll.", r"$<$5\,m", "Min gap (m)", r"Work", r"Flow",
              r"Work", r"Flow"]
    lines = [r"\begin{table}[htbp]", r"\centering", r"\footnotesize",
             r"\caption{Every candidate controller, in the order it was tried. Each was run 16 times (4 drives "
             r"$\times$ 4 penetration rates). Coll.: runs that ended in a collision. $<$5\,m: runs in which a "
             r"smart car came closer than the 5\,m minimum gap. Min gap: closest approach of any smart car. Work "
             r"and flow: change in \%, averaged over drives and penetration rates, against the all-IDM baseline "
             r"and against TG ($\tau=2.5$\,s) at the same penetration rate; -- if any run crashed.}",
             r"\label{tab:path}", r"\setlength{\tabcolsep}{4pt}", r"\begin{tabular}{llp{4.6cm}rrrrrrr}",
             r"\toprule", r" & & & & & & \multicolumn{2}{c}{vs IDM (\%)} & \multicolumn{2}{c}{vs TG (\%)} \\",
             r"\cmidrule(lr){7-8}\cmidrule(lr){9-10}", " & ".join(header) + r" \\", r"\midrule"]
    lines += [" & ".join(r) + r" \\" for r in rows]
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    TEX.mkdir(parents=True, exist_ok=True)
    (TEX / "iteration_path.tex").write_text("\n".join(lines), encoding="utf-8")


def tex_final_vs_alternatives(study):
    """Work and flow change vs IDM per penetration rate: TG, WTJ4, WTJSoftLB beta=1, STG."""
    cands = [(TG, "TG"), (WTJ4, "WTJ4"), (BETA1, r"$\beta=1$"), (STG, r"\textbf{STG}")]
    cols = ["avg_positive_work", "flow_mean_veh_per_hr"]
    rows = []
    for p in RATES:
        rows.append([f"{p:g}"] + [fmt(study.change(c, p, col)) for col in cols for c, _ in cands])
    rows.append(["Mean"] + [fmt(mean_change(study, c, col)) for col in cols for c, _ in cands])
    rows.append([r"$<$5\,m"] + [f"{study.safety(c)[1]}/{study.safety(c)[2]}" for c, _ in cands] + [""] * 4)
    for r in rows:   # STG columns in bold
        r[4], r[8] = rf"\textbf{{{r[4]}}}", rf"\textbf{{{r[8]}}}" if r[8] else ""
    names = " & ".join(n for _, n in cands)
    lines = [r"\begin{table}[htbp]", r"\centering", r"\small",
             r"\caption{The final controller (STG, $\lambda=2$, $\tau=2.5$\,s) against the alternatives, by "
             r"penetration rate $p$: change in work and flow from the all-IDM baseline (\%), averaged over the "
             r"four drives. TG: time gap controller, $\tau=2.5$\,s. WTJ4: WTJSoftLowerBound, $\beta=0.25$. "
             r"$\beta=1$: WTJSoftLowerBound, $\beta=1$. Last row: runs (of 16) in which a smart car came closer "
             r"than 5\,m.}",
             r"\label{tab:compare}", r"\begin{tabular}{lrrrrrrrr}", r"\toprule",
             r" & \multicolumn{4}{c}{Work (\%)} & \multicolumn{4}{c}{Flow (\%)} \\",
             r"\cmidrule(lr){2-5}\cmidrule(lr){6-9}", rf"$p$ & {names} & {names} \\", r"\midrule"]
    for i, r in enumerate(rows):
        if i in (len(RATES), len(RATES) + 1):
            lines.append(r"\midrule")
        lines.append(" & ".join(r) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    (TEX / "final_vs_alternatives.tex").write_text("\n".join(lines), encoding="utf-8")


def tex_tuning(study):
    """tau (STG and TG) and lam (STG) sweeps: safety, mean work and flow, and work at p = 1."""
    W, F = "avg_positive_work", "flow_mean_veh_per_hr"
    groups = [(r"Time gap $\tau$, STG ($\lambda=2$)", [(rf"$\tau={t:g}$\,s", TAU_GRID[("STG", t)]) for t in TAUS]),
              (r"Time gap $\tau$, TG ($\lambda=0.1$, two-sided)",
               [(rf"$\tau={t:g}$\,s", TAU_GRID[("TG", t)]) for t in TAUS]),
              (r"Damping $\lambda$, STG ($\tau=2.5$\,s)", [(rf"$\lambda={l:g}$", LAM_GRID[l]) for l in LAMS])]
    lines = [r"\begin{table}[htbp]", r"\centering", r"\small",
             r"\caption{Tuning the time gap $\tau$ and the damping gain $\lambda$. Coll.\ and $<$5\,m: runs (of "
             r"16) with a collision or with a smart car closer than 5\,m. Work and flow: change from the "
             r"all-IDM baseline (\%), averaged over drives and penetration rates; Work, $p=1$: the same for an "
             r"all-smart platoon only. Bold: the final setting.}", r"\label{tab:tuning}", r"\begin{tabular}{lrrrrr}",
             r"\toprule", r"Setting & Coll. & $<$5\,m & Work (\%) & Work, $p=1$ (\%) & Flow (\%) \\"]
    for title, items in groups:
        lines += [r"\midrule", rf"\multicolumn{{6}}{{l}}{{\emph{{{title}}}}} \\"]
        for name, cfg in items:
            crash, inside, n, _ = study.safety(cfg)
            row = [name, f"{crash}/{n}", f"{inside}/{n}", fmt(mean_change(study, cfg, W)),
                   fmt(study.change(cfg, 1.0, W)), fmt(mean_change(study, cfg, F))]
            lines.append(" & ".join(bold(row) if cfg == STG else row) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    (TEX / "tuning.tex").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Reproduce the controller iteration: runs and report tables.")
    parser.add_argument("--results-dir", help="runs.csv + runs/ folder (default: $WTJ_RESULTS_DIR or results/)")
    parser.add_argument("--no-run", action="store_true", help="don't run missing simulations")
    parser.add_argument("--dry-run", action="store_true", help="only list the simulations that are missing")
    parser.add_argument("--jobs", type=int, default=8, help="simulations at once (default 8)")
    parser.add_argument("--csv", action="store_true", help="also write CSV tables to results/iteration/")
    args = parser.parse_args()
    sweep_data.EXPORT_CSV = args.csv

    results = results_dir(args.results_dir)
    bags = all_bags()
    print(f"Results: {results}\nBags: {', '.join(bags)}")
    if not args.no_run:
        n = ensure_runs(results, ALL_CONFIGS, bags, jobs=args.jobs, dry_run=args.dry_run)
        print(f"{n} simulation(s) {'missing' if args.dry_run else 'run'}")
        if args.dry_run:
            return

    study = Study(results, ALL_CONFIGS, bags)
    missing = [(c.label, b, p) for (c, b, p), r in study.rows.items() if r is None]
    if missing:
        print(f"Note: {len(missing)} run(s) missing, e.g. {missing[:3]}; they show as --")

    print("Tables")
    write_csv(TABLES / "all_runs.csv", study.records())
    # Per-stage CSV tables (with --csv)
    write_csv(TABLES / "stage1_2_wtj_variants.csv", summary_rows(study, STAGE12))
    write_csv(TABLES / "stage3_beta_sweep.csv",
              summary_rows(study, [BETA_GRID[(t, b)] for t in TAU_MINS for b in BETAS],
                           {BETA_GRID[(t, b)]: f"tau_min={t:g}, beta={b:g}" for t in TAU_MINS for b in BETAS}))
    write_csv(TABLES / "stage4_stg.csv", summary_rows(study, STAGE4))
    write_csv(TABLES / "stage5_tau_sweep.csv", summary_rows(study, [TAU_GRID[(c, t)] for c in ("TG", "STG")
                                                                    for t in TAUS]))
    write_csv(TABLES / "stage6_lam_sweep.csv", summary_rows(study, [LAM_GRID[l] for l in LAMS]))

    # LaTeX tables for report/main.tex
    tex_iteration_path(study)
    tex_final_vs_alternatives(study)
    tex_tuning(study)

    print(f"LaTeX tables: {TEX.relative_to(ROOT)}" + (f"; CSV tables: {TABLES.relative_to(ROOT)}" if args.csv else ""))


if __name__ == "__main__":
    main()
