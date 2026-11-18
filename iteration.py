"""
Reproduce the controller iteration, from the original WTJ controller to the
Special Time Gap (STG) controller: every simulation it needs (only the missing
ones are run), then the figures, CSV tables and LaTeX tables for the report.

    uv run iteration.py --results-dir C:/Users/me/wtj_results          # run what's missing, then plot
    uv run iteration.py --results-dir ... --plots-only                  # only plot saved runs
    uv run iteration.py --results-dir ... --dry-run                     # list what would be run

Stages (seed 7, 100 vehicles, IDM base, every bag, p = 0.25 0.5 0.75 1, plus
the all-IDM p = 0 baseline of each bag):

  1. Original WTJ:             WTJ0 (lam 0.5), WTJ1 (lam 2), with TG for reference
  2. Soft lower bound:         WTJ2 (beta 0), WTJ3 (beta 0.1), WTJ4 (beta 0.25)
  3. Beta sweep:               WTJSoftLowerBound, beta 0-1, tau_min 2 and 2.5
  4. Special time gap:         STG lam 0.1 and lam 2, against TG, WTJ4 and beta = 1
  5. Time gap sweep:           TG and STG, tau = 2, 2.5, 3, 4
  6. Damping sweep:            STG (tau 2.5), lam = 1, 2, 3, 4; and the lam >= 3 oscillation

Outputs:
  plots/iteration/fig*.png               figures
  results/iteration/*.csv                with --csv: one table per stage, plus every run (all_runs.csv)
  report/tables/*.tex                    the tables as LaTeX (booktabs), for report/main.tex
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from run_io import bag_label, load_run, repair_csv
import sweep_data
from sweep_data import (BASELINE, METRICS, RATES, Config, Study, all_bags, ensure_runs, results_dir,
                        write_csv)

ROOT = Path(__file__).parent
PLOTS = ROOT / "plots" / "iteration"
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

# --- style -----------------------------------------------------------------------------------
SURFACE, GRID, TEXT2 = "#fcfcfb", "#e4e3df", "#5c5b57"
UNSAFE = "#c2410c"
COLORS = {"WTJ0": "#2a78d6", "WTJ1": "#1baf7a", "WTJ2": "#6250d6", "WTJ3": "#008300", "WTJ4": "#eda100",
          "TG": "#e34948", "STG": "#8f1d1b", "STG lam=0.1": "#e87ba4", "beta1": "#0e7490"}
MARKERS = {"WTJ0": "o", "WTJ1": "s", "WTJ2": "v", "WTJ3": "p", "WTJ4": "D", "TG": "^", "STG": "<",
           "STG lam=0.1": "P", "beta1": "h"}
P_COLORS = {0.25: "#9ec5f0", 0.5: "#5a9be0", 0.75: "#2a78d6", 1.0: "#173f75"}


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, linewidth=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=TEXT2)
    ax.axhline(0, color=TEXT2, linewidth=0.8)


def save(fig, name):
    PLOTS.mkdir(parents=True, exist_ok=True)
    path = PLOTS / name
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(f"  {path.relative_to(ROOT)}")


def fmt(v, digits=1, signed=True):
    if v is None or v == "" or (isinstance(v, float) and np.isnan(v)):
        return "--"
    if not signed:
        return f"{v:.{digits}f}"
    s = f"{v:+.{digits}f}"
    return f"${s[0]}${s[1:]}"   # LaTeX minus/plus sign, digits stay in text (so \textbf applies)


def latex_table(name, header, rows, caption, label, align=None):
    """Write a booktabs table to report/tables/<name>.tex."""
    TEX.mkdir(parents=True, exist_ok=True)
    align = align or "l" + "r" * (len(header) - 1)
    esc = lambda s: str(s).replace("%", r"\%").replace("_", r"\_").replace("--", "--")
    lines = [r"\begin{table}[htbp]", r"\centering", r"\small", rf"\caption{{{caption}}}", rf"\label{{{label}}}",
             rf"\begin{{tabular}}{{{align}}}", r"\toprule", " & ".join(header) + r" \\", r"\midrule"]
    for r in rows:
        lines.append(r"\midrule" if r == "MIDRULE" else " & ".join(esc(c) for c in r) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    (TEX / f"{name}.tex").write_text("\n".join(lines), encoding="utf-8")


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


def mean_or_none(rec, col):
    v = rec[f"{col}_mean_pct"]
    return None if v == "" else float(v)


# --- figures ---------------------------------------------------------------------------------
def fig_stage12(study):
    """Work and avg flow (change vs IDM) against p, one column per bag, WTJ0-WTJ4 and TG."""
    fig, axes = plt.subplots(2, len(study.bags), figsize=(5.2 * len(study.bags), 9), facecolor=SURFACE,
                             sharex=True)
    for j, bag in enumerate(study.bags):
        for i, (col, lab) in enumerate(METRICS[:2]):
            ax = axes[i][j]
            style(ax)
            for cfg in STAGE12:
                key = cfg.label
                xs, ys = [], []
                for p in study.rates:
                    m = study.metrics(cfg, bag, p)
                    if m is None:
                        continue
                    if m["status"] == "collision":
                        ax.scatter([p], [0], marker="x", s=90, color=COLORS[key], linewidths=2.2, zorder=5)
                        xs.append(p), ys.append(np.nan)
                        continue
                    v = 100 * (float(m[col]) / float(study.metrics(None, bag, 0)[col]) - 1)
                    xs.append(p), ys.append(v)
                    if m["status"].startswith("inside"):
                        ax.scatter([p], [v], s=300, facecolors="none", edgecolors=UNSAFE, linewidths=1.6, zorder=4)
                ax.plot(xs, ys, color=COLORS[key], marker=MARKERS[key], markersize=7, linewidth=1.8, label=key)
            ax.set_xticks(study.rates)
            if i == 0:
                ax.set_title(bag_label(bag), fontsize=11)
            if j == 0:
                ax.set_ylabel(f"{lab}: change vs all-IDM (%)")
            if i == 1:
                ax.set_xlabel("Penetration rate")
    handles, labels = axes[0][0].get_legend_handles_labels()
    handles += [plt.Line2D([], [], marker="x", color=TEXT2, linestyle="", markersize=9, markeredgewidth=2),
                plt.Line2D([], [], marker="o", markerfacecolor="none", markeredgecolor=UNSAFE, linestyle="",
                           markersize=13)]
    labels += ["collision (run stopped)", "smart car inside 5 m"]
    fig.legend(handles, labels, loc="lower center", ncol=8, frameon=False, bbox_to_anchor=(0.5, -0.03))
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    save(fig, "fig01_wtj_variants.png")


def fig_wtj2_lag(study, bag="wtj_real_drive", p=1.0):
    """Platoon length (leader to last car) over time: WTJ2's cars stop following the car ahead."""
    fig, ax = plt.subplots(figsize=(11, 4.8), facecolor=SURFACE)
    style(ax)
    curves = [(None, BASELINE, TEXT2), (WTJ2, "WTJ2 (beta 0)", COLORS["WTJ2"]), (WTJ4, "WTJ4 (beta 0.25)",
              COLORS["WTJ4"]), (TG, "TG", COLORS["TG"])]
    rows = []
    for cfg, lab, color in curves:
        d = study.run_dir(cfg, bag, p)
        if d is None:
            continue
        run = load_run(d)
        real = run.rows(ghost=False)
        length = run.position[real].max(axis=0) - run.position[real].min(axis=0)
        ax.plot(run.time, length / 1000, color=color, linewidth=1.6, label=lab)
        rows += [{"controller": lab, "time_s": t, "platoon_length_m": round(float(l), 1)}
                 for t, l in zip(run.time[::20], length[::20])]
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Platoon length, leader to last car (km)")
    ax.set_title(f"{bag_label(bag)}, p = {p:g}: platoon length", loc="left", fontsize=11)
    ax.legend(frameon=False)
    save(fig, "fig02_wtj2_platoon_length.png")
    write_csv(TABLES / "fig02_platoon_length.csv", rows)


def fig_beta(study):
    """Work / flow / throughput against beta, per p; tau_min 2 and 2.5; TG as reference."""
    fig, axes = plt.subplots(3, len(study.rates), figsize=(20, 12), facecolor=SURFACE, sharex=True)
    tau_style = {2.0: ("#eda100", "tau_min = 2"), 2.5: ("#2a78d6", "tau_min = 2.5")}
    for i, (col, lab) in enumerate(METRICS):
        for j, p in enumerate(study.rates):
            ax = axes[i][j]
            style(ax)
            for t in TAU_MINS:
                color, name = tau_style[t]
                ys = []
                for b in BETAS:
                    cfg = BETA_GRID[(t, b)]
                    crashed = any(study.metrics(cfg, bag, p) and study.metrics(cfg, bag, p)["status"] == "collision"
                                  for bag in study.bags)
                    if crashed:
                        ax.scatter([b], [0], marker="x", s=70, color=color, linewidths=2, zorder=5)
                    ys.append(study.change(cfg, p, col))
                ax.plot(BETAS, [np.nan if y is None else y for y in ys], color=color, marker="o", markersize=4.5,
                        linewidth=2, label=f"WTJSoftLowerBound, {name}")
            tg = study.change(TG, p, col)
            if tg is not None:
                ax.axhline(tg, color=COLORS["TG"], linewidth=1.4, linestyle=(0, (2, 2)), label="TG (same p)")
            ax.axvline(0.25, color=TEXT2, linewidth=0.8, linestyle=":")
            if i == 0:
                ax.set_title(f"p = {p:g}", fontsize=12)
            if j == 0:
                ax.set_ylabel(f"{lab}: change vs all-IDM (%)")
            if i == 2:
                ax.set_xlabel("beta")
    axes[0][0].legend(frameon=False, fontsize=9, loc="lower left")
    fig.suptitle("WTJSoftLowerBound (lam = 3, tau_max = 3) against beta, mean over bags. x: a bag crashed. "
                 "Dotted: WTJ4 (beta = 0.25)", x=0.01, ha="left", fontsize=12)
    fig.tight_layout()
    save(fig, "fig03_beta_sweep.png")


def fig_stg(study):
    """Work and flow against p (mean over bags) for TG, STG lam 0.1, STG lam 2, WTJ4 and beta = 1."""
    names = {TG: "TG", STG_LAM01: "STG lam=0.1", STG: "STG", WTJ4: "WTJ4", BETA1: "beta1"}
    labels = {TG: "TG (tau 2.5, two-sided lam 0.1)", STG_LAM01: "STG, lam = 0.1",
              STG: "STG, lam = 2 (braking only)", WTJ4: "WTJSoftLB beta = 0.25 (WTJ4)",
              BETA1: "WTJSoftLB beta = 1"}
    fig, axes = plt.subplots(1, 3, figsize=(19, 5.6), facecolor=SURFACE)
    for ax, (col, lab) in zip(axes, METRICS):
        style(ax)
        for cfg in STAGE4:
            ys = [study.change(cfg, p, col) for p in study.rates]
            ax.plot(study.rates, [np.nan if y is None else y for y in ys], color=COLORS[names[cfg]],
                    marker=MARKERS[names[cfg]], markersize=8, linewidth=2, label=labels[cfg])
            for p, y in zip(study.rates, ys):
                if y is not None and study.safety(cfg, [p])[1]:
                    ax.scatter([p], [y], s=300, facecolors="none", edgecolors=UNSAFE, linewidths=1.6, zorder=4)
        ax.set_xticks(study.rates)
        ax.set_xlabel("Penetration rate")
        ax.set_title(f"{lab}: change vs all-IDM (%)", loc="left", fontsize=11)
    axes[0].legend(frameon=False, fontsize=9)
    fig.suptitle("Special time gap controller against TG and WTJSoftLowerBound, mean over bags. "
                 "Ring: a smart car got inside 5 m on some bag", x=0.01, ha="left", fontsize=12)
    fig.tight_layout()
    save(fig, "fig04_stg_vs_tg_wtj.png")


def sweep_figure(study, xs, grid, key_of, xlabel, title, name, refs=()):
    """Rows = metrics, columns = p, x = swept value; one line per controller in grid {(ctrl, x): cfg}."""
    ctrls = list(dict.fromkeys(c for c, _ in grid))
    fig, axes = plt.subplots(3, len(study.rates), figsize=(20, 12), facecolor=SURFACE, sharex=True)
    for i, (col, lab) in enumerate(METRICS):
        for j, p in enumerate(study.rates):
            ax = axes[i][j]
            style(ax)
            for c in ctrls:
                ys = []
                for x in xs:
                    cfg = grid[(c, x)]
                    crash, inside, _, _ = study.safety(cfg, [p])
                    y = study.change(cfg, p, col)
                    ys.append(np.nan if y is None else y)
                    if crash:
                        ax.scatter([x], [0], marker="x", s=80, color=COLORS[c], linewidths=2, zorder=5)
                    elif inside:
                        ax.scatter([x], [y], s=280, facecolors="none", edgecolors=UNSAFE, linewidths=1.6, zorder=4)
                ax.plot(xs, ys, color=COLORS[c], marker=MARKERS[c], markersize=8, linewidth=2, label=key_of(c))
            for cfg, lab_, color in refs:
                v = study.change(cfg, p, col)
                if v is not None:
                    ax.axhline(v, color=color, linewidth=1.4, linestyle=(0, (2, 2)), label=lab_)
            ax.set_xticks(xs)
            if i == 0:
                ax.set_title(f"p = {p:g}", fontsize=12)
            if j == 0:
                ax.set_ylabel(f"{lab}: change vs all-IDM (%)")
            if i == 2:
                ax.set_xlabel(xlabel)
    handles, labels = axes[0][0].get_legend_handles_labels()
    handles += [plt.Line2D([], [], marker="o", markerfacecolor="none", markeredgecolor=UNSAFE, linestyle="",
                           markersize=13), plt.Line2D([], [], marker="x", color=TEXT2, linestyle="", markersize=9)]
    labels += ["a bag got inside 5 m", "a bag crashed (no mean)"]
    axes[0][0].legend(handles, labels, frameon=False, fontsize=9)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12)
    fig.tight_layout()
    save(fig, name)


def fig_lam_instability(study, results, bag="wtj_real_drive", p=1.0):
    """(a) work per car along the platoon at p = 1 for each lam; (b) acceleration of car 90, lam 2 vs 4."""
    runs = {l: study.row(LAM_GRID[l], bag, p) for l in LAMS}
    work = {}
    repair_csv(results / "cars.csv")
    with open(results / "cars.csv", newline="") as f:
        names = {r["run"]: l for l, r in runs.items() if r}
        for c in csv.DictReader(f):
            if c["run"] in names:
                work.setdefault(names[c["run"]], {})[int(c["car"])] = float(c["avg_positive_work"])
    fig, (a, b) = plt.subplots(1, 2, figsize=(17, 5.4), facecolor=SURFACE)
    style(a), style(b)
    lam_colors = {1.0: "#f0a3a3", 2.0: "#8f1d1b", 3.0: "#d97706", 4.0: "#1f2937"}
    rows = []
    for l in LAMS:
        w = work.get(l, {})
        cars = sorted(w)
        a.plot(cars, [w[k] for k in cars], color=lam_colors[l], linewidth=1.8, label=f"lam = {l:g}")
        rows += [{"lam": l, "car": k, "avg_positive_work": w[k]} for k in cars]
    a.set_xlabel("Car number (1 = behind the leader)")
    a.set_ylabel("Average positive work (m²/s³)")
    a.set_title(f"(a) Work along the platoon, {bag_label(bag)}, p = {p:g}", loc="left", fontsize=11)
    a.legend(frameon=False)
    write_csv(TABLES / "fig07_work_per_car.csv", rows)
    acc_rows = []
    for l in (2.0, 4.0):
        d = study.run_dir(LAM_GRID[l], bag, p)
        if d is None:
            continue
        run = load_run(d)
        k = next(i for i, v in enumerate(run.vehicles) if v["vehicle"] == 90 and not v["ghost"])
        win = (run.time >= 400) & (run.time <= 520)
        b.plot(run.time[win], run.acceleration[k, win], color=lam_colors[l], linewidth=1.4, label=f"lam = {l:g}")
        acc_rows += [{"lam": l, "time_s": round(float(t), 2), "accel_mps2": round(float(x), 4)}
                     for t, x in zip(run.time[win], run.acceleration[k, win])]
    b.set_xlabel("Time (s)")
    b.set_ylabel("Acceleration of car 90 (m/s²)")
    b.set_title("(b) Car 90 acceleration, 400-520 s", loc="left", fontsize=11)
    b.legend(frameon=False)
    write_csv(TABLES / "fig07_car90_accel.csv", acc_rows)
    fig.tight_layout()
    save(fig, "fig07_lam_instability.png")


# --- tables ----------------------------------------------------------------------------------
# --- report tables (main.tex uses these) -----------------------------------------------------
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
    parser = argparse.ArgumentParser(description="Reproduce the controller iteration: runs, figures, tables.")
    parser.add_argument("--results-dir", help="runs.csv + runs/ folder (default: $WTJ_RESULTS_DIR or results/)")
    parser.add_argument("--plots-only", action="store_true", help="don't run missing simulations")
    parser.add_argument("--dry-run", action="store_true", help="only list the simulations that are missing")
    parser.add_argument("--jobs", type=int, default=8, help="simulations at once (default 8)")
    parser.add_argument("--csv", action="store_true", help="also write CSV tables to results/iteration/")
    args = parser.parse_args()
    sweep_data.EXPORT_CSV = args.csv

    results = results_dir(args.results_dir)
    bags = all_bags()
    print(f"Results: {results}\nBags: {', '.join(bags)}")
    if not args.plots_only:
        n = ensure_runs(results, ALL_CONFIGS, bags, jobs=args.jobs, dry_run=args.dry_run)
        print(f"{n} simulation(s) {'missing' if args.dry_run else 'run'}")
        if args.dry_run:
            return

    study = Study(results, ALL_CONFIGS, bags)
    missing = [(c.label, b, p) for (c, b, p), r in study.rows.items() if r is None]
    if missing:
        print(f"Note: {len(missing)} run(s) missing, e.g. {missing[:3]}; their points are left out")

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
    for old in TEX.glob("stage*.tex"):   # per-stage tables of earlier versions
        old.unlink()
    tex_iteration_path(study)
    tex_final_vs_alternatives(study)
    tex_tuning(study)

    print("Figures")
    fig_stage12(study)
    fig_wtj2_lag(study)
    fig_beta(study)
    fig_stg(study)
    sweep_figure(study, TAUS, TAU_GRID, lambda c: c + (" (lam 0.1, two-sided)" if c == "TG" else
                                                       " (lam 2, braking only)"), "tau (s)",
                 "TG and STG against the time gap tau, mean over bags. Dotted: tau = 2.5", "fig05_tau_sweep.png")
    sweep_figure(study, LAMS, {("STG", l): LAM_GRID[l] for l in LAMS}, lambda c: "STG, tau = 2.5",
                 "lam (braking-only damping gain)", "STG (tau = 2.5) against lam, mean over bags",
                 "fig06_lam_sweep.png", refs=[(TG, "TG, tau = 2.5", COLORS["TG"])])
    fig_lam_instability(study, results)
    print(f"LaTeX tables: {TEX.relative_to(ROOT)}" + (f"; CSV tables: {TABLES.relative_to(ROOT)}" if args.csv else ""))


if __name__ == "__main__":
    main()
