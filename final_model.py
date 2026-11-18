"""
Figures and data for the final controller: STG (special time gap, braking-only
damping lam = 2) against TG at the same time gap tau, on every bag at every
penetration rate. Missing simulations are run first.

    uv run final_model.py --results-dir C:/Users/me/wtj_results                 # tau = 2.5
    uv run final_model.py --results-dir ... --tau 2 2.5 3                         # several time gaps
    uv run final_model.py --results-dir ... --plots-only --jobs 4

Grids with one row per bag and one column per penetration rate:

  fundamental_diagram.png                flow vs density boxes, STG and TG, with the all-IDM baseline
  cumulative_work_STG.png, _TG.png        accumulated positive work of every vehicle over time
  cumulative_work_mean.png                platoon mean of the above, STG, TG and the all-IDM baseline

and the report's figures, on the WTJ real drive:

  report_time_space.png     all-IDM baseline, then STG at every penetration rate (TG looks the same)
  report_fd.png             fundamental diagram per penetration rate, STG, TG and the all-IDM baseline
  report_work.png           platoon-mean accumulated work per penetration rate, with the 10-90 % range

Figures go to plots/final_model/tau<tau>/.

CSV files, only with --csv (one row per item, so they can be filtered and joined):

  results/final_model/tau<tau>/          (small, kept in the repo)
    summary.csv                 per bag, p, controller: status, work, flow, throughput, closest gaps,
                                and STG's change vs TG
    fd_boxes.csv                every density/flow box (bag, p, controller, t0, x0, density, flow, speed)
    work_per_vehicle.csv        every vehicle's total positive work and type
  <results-dir>/final_model/tau<tau>/   (large, beside the runs)
    time_space_<bag>_p<p>_<controller>.csv       position and speed of every vehicle every --sample s
    cumulative_work_<bag>_p<p>_<controller>.csv  accumulated positive work of every vehicle every --sample s

Work is sum max(a v, 0) dt per vehicle (m^2/s^2, no mass); runs.csv's
avg_positive_work is that total divided by the run length.
"""

import argparse
import csv
import shutil
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from density_flow import density_and_flow_for
from run_io import BAG_LABELS, bag_label, load_run
import sweep_data
from sweep_data import BASELINE, RATES, Config, Study, all_bags, ensure_runs, results_dir, write_csv

ROOT = Path(__file__).parent
SURFACE, GRID, TEXT2 = "#fcfcfb", "#e4e3df", "#5c5b57"
COLOR = {"STG": "#8f1d1b", "TG": "#e34948", BASELINE: "#8a8985"}
IDM_GREY = "#b9b8b3"
CMAP = "RdYlGn"


def configs(tau, lam):
    return Config("STG", {"tau": tau, "lam": lam}, "STG"), Config("TG", {"tau": tau}, "TG")


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, linewidth=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=TEXT2)


def real_rows(run):
    """Indices of leader + followers in platoon order, and the followers' types."""
    rows = run.rows(ghost=False)
    return rows, [run.vehicles[k]["type"] for k in rows]


def cumulative_work(run, rows):
    """(n_rows, n_steps) accumulated positive work, m^2/s^2."""
    p = np.maximum(run.acceleration[rows] * run.velocity[rows], 0) * run.dt
    return np.cumsum(p, axis=1)


def status_note(m):
    if m is None:
        return "not run"
    if m["status"] == "collision":
        return f"COLLISION - {m['collision']}"
    return m["status"] + (f" (closest {m['min_gap_smart_m']} m)" if m["status"].startswith("inside") else "")


# --- per run (worker): extract what the figures need, write that run's large CSVs ------------
TS_POINTS = 1000   # time samples per trajectory in a time-space panel


def extract(job):
    """Load one run and return its decimated trajectories, accumulated work and FD boxes."""
    (bag, p, ctrl, run_dir, tau, big, sample, export) = job
    run = load_run(run_dir)
    rows, types = real_rows(run)
    smart = np.array([t == ctrl for t in types])
    stride = max(1, len(run.time) // TS_POINTS)
    step = max(1, round(sample / run.dt))
    cw = cumulative_work(run, rows)
    boxes = density_and_flow_for(Path(run_dir))
    out = {"bag": bag, "p": p, "ctrl": ctrl, "smart": smart,
           "ts_t": run.time[::stride].astype(np.float32),
           "ts_x": (run.position[rows, ::stride] / 1000).astype(np.float32),
           "ts_v": run.velocity[rows, ::stride].astype(np.float32),
           "vmax": float(np.percentile(run.velocity[rows], 99.5)),
           "cw_t": run.time[::step].astype(np.float32), "cw": cw[:, ::step].astype(np.float32),
           "fd": (np.asarray(boxes["density"]), np.asarray(boxes["flow"])), "fd_rows": [], "veh_rows": []}
    if ctrl == BASELINE:
        return out
    out["fd_rows"] = [{"bag": bag, "penetration_rate": p, "tau": tau, "controller": ctrl, "t0_s": t0, "x0_m": x0,
                       "density_veh_per_km": round(float(k), 3), "flow_veh_per_hr": round(float(q), 2),
                       "speed_mps": round(float(v), 3)}
                      for t0, x0, k, q, v in zip(boxes["t0"], boxes["x0"], boxes["density"], boxes["flow"],
                                                 boxes["speed"])]
    for i, k in enumerate(rows[1:], start=1):
        out["veh_rows"].append({"bag": bag, "penetration_rate": p, "tau": tau, "controller": ctrl,
                                "vehicle": run.vehicles[k]["vehicle"], "type": types[i],
                                "mass_kg": run.vehicles[k]["mass_kg"],
                                "total_positive_work_m2ps2": round(float(cw[i, -1]), 4),
                                "avg_positive_work_m2ps3": round(float(cw[i, -1] / (len(run.time) * run.dt)), 5)})
    if export:
        name = f"{bag}_p{p:g}_{ctrl}.csv"
        write_rows(big / f"time_space_{name}", ["vehicle", "type", "time_s", "position_m", "velocity_mps"],
                   [(run.vehicles[k]["vehicle"], types[i], round(float(t), 2), round(float(x), 2), round(float(v), 3))
                    for i, k in enumerate(rows)
                    for t, x, v in zip(run.time[::step], run.position[k, ::step], run.velocity[k, ::step])])
        write_rows(big / f"cumulative_work_{name}", ["vehicle", "type", "time_s", "cumulative_positive_work_m2ps2"],
                   [(run.vehicles[k]["vehicle"], types[i], round(float(t), 2), round(float(w), 4))
                    for i, k in enumerate(rows) if i
                    for t, w in zip(run.time[::step], cw[i, ::step])])
    return out


# --- grid figures: one row per bag, one column per penetration rate ---------------------------
def grid(bags, ncols, title, w=4.3, h=3.1, layout=None):
    fig, axes = plt.subplots(len(bags), ncols, figsize=(w * ncols, h * len(bags)), facecolor=SURFACE,
                             squeeze=False, sharex="row", sharey="row", layout=layout)
    for ax in axes.flat:
        style(ax)
        ax.tick_params(labelsize=8)
    fig.suptitle(title, x=0.01, ha="left", fontsize=14)
    return fig, axes


def label_grid(axes, bags, cols, ylabel, xlabel="Time (s)"):
    for j, p in enumerate(cols):
        axes[0, j].set_title("all-IDM baseline (p = 0)" if p == 0 else f"p = {p:g}", fontsize=11)
        axes[-1, j].set_xlabel(xlabel, fontsize=9)
    for i, bag in enumerate(bags):
        axes[i, 0].set_ylabel(f"{bag_label(bag)}\n{ylabel}", fontsize=9)


def note(ax, m, missing="no trajectories"):
    """Mark a panel whose run crashed, got inside 5 m, or is missing."""
    if m is None or m["status"] == "collision":
        ax.text(0.5, 0.5, status_note(m) if m else missing, transform=ax.transAxes, ha="center", va="center",
                color=COLOR["STG"], fontsize=9, wrap=True)
    elif m["status"].startswith("inside"):
        ax.text(0.98, 0.03, f"inside 5 m (closest {m['min_gap_smart_m']} m)", transform=ax.transAxes,
                ha="right", va="bottom", color=COLOR["STG"], fontsize=8)


def fig_legend(fig, axes):
    """One legend for the whole grid, top right, out of the data."""
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", ncol=len(labels), frameon=False, fontsize=11, markerscale=2,
               bbox_to_anchor=(1, 1))


def save_grid(fig, path):
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)


def draw_fd(data, bags, tau, out):
    fig, axes = grid(bags, len(RATES), f"Fundamental diagrams, STG vs TG, tau = {tau:g} s "
                                       f"(250 m x 30 s boxes inside the platoon)")
    for i, bag in enumerate(bags):
        for j, p in enumerate(RATES):
            ax = axes[i, j]
            for c, s, a in ((BASELINE, 10, 0.4), ("TG", 14, 0.8), ("STG", 14, 0.8)):
                d = data.get((bag, 0.0 if c == BASELINE else p, c))
                if d is not None:
                    ax.scatter(*d["fd"], s=s, color=COLOR[c], alpha=a, edgecolors="none", label=c)
            ax.set_xlim(left=0)
            ax.set_ylim(bottom=0)
    fig_legend(fig, axes)
    label_grid(axes, bags, RATES, "Flow (veh/h)", "Density (veh/km)")
    save_grid(fig, out / "fundamental_diagram.png")


def draw_work(data, metrics, bags, ctrl, tau, out):
    fig, axes = grid(bags, len(RATES), f"Accumulated positive work of every vehicle, {ctrl}, tau = {tau:g} s "
                                       f"({ctrl} cars colored, IDM cars grey, black: platoon mean)")
    for i, bag in enumerate(bags):
        for j, p in enumerate(RATES):
            ax = axes[i, j]
            note(ax, metrics.get((bag, p, ctrl)))
            d = data.get((bag, p, ctrl))
            if d is None:
                continue
            t, cw, smart = d["cw_t"], d["cw"][1:], d["smart"][1:]
            for sel, color, lw in ((~smart, IDM_GREY, 0.5), (smart, COLOR[ctrl], 0.7)):
                if sel.any():
                    ax.add_collection(LineCollection(np.stack([np.broadcast_to(t, cw[sel].shape), cw[sel]], -1),
                                                     colors=color, linewidths=lw, alpha=0.7, rasterized=True))
            ax.plot(t, cw.mean(axis=0), color="black", linewidth=1.8)
            ax.autoscale()
            ax.set_ylim(bottom=0)
    label_grid(axes, bags, RATES, "Work (m²/s²)")
    save_grid(fig, out / f"cumulative_work_{ctrl}.png")


def draw_work_mean(data, bags, tau, out):
    fig, axes = grid(bags, len(RATES), f"Platoon-mean accumulated positive work, STG vs TG, tau = {tau:g} s")
    for i, bag in enumerate(bags):
        base = data.get((bag, 0.0, BASELINE))
        for j, p in enumerate(RATES):
            ax = axes[i, j]
            if base is not None:
                ax.plot(base["cw_t"], base["cw"][1:].mean(axis=0), color=COLOR[BASELINE], linewidth=1.8,
                        linestyle=(0, (5, 3)), label="all-IDM baseline")
            for c in ("TG", "STG"):
                d = data.get((bag, p, c))
                if d is not None:
                    ax.plot(d["cw_t"], d["cw"][1:].mean(axis=0), color=COLOR[c], linewidth=1.8, label=c)
            ax.set_ylim(bottom=0)
    fig_legend(fig, axes)
    label_grid(axes, bags, RATES, "Work (m²/s²)")
    save_grid(fig, out / "cumulative_work_mean.png")


# --- the three report figures: one bag, compact -----------------------------------------------
REPORT_BAG = "wtj_real_drive"


def add_trajectories(ax, d, norm, linewidth=(0.35, 0.8)):
    """Every vehicle's trajectory colored by speed; smart cars thicker."""
    pts = np.stack([np.broadcast_to(d["ts_t"], d["ts_x"].shape), d["ts_x"]], axis=-1)
    segs = np.stack([pts[:, :-1], pts[:, 1:]], axis=2).reshape(-1, 2, 2)
    widths = np.repeat(np.where(np.r_[False, d["smart"][1:]], linewidth[1], linewidth[0]), pts.shape[1] - 1)
    lc = LineCollection(segs, cmap=CMAP, norm=norm, linewidths=widths, rasterized=True)
    lc.set_array(d["ts_v"][:, :-1].ravel())
    ax.add_collection(lc)
    ax.autoscale()


def draw_report_figures(data, metrics, tau, lam, out, bag=REPORT_BAG):
    """report_time_space.png, report_fd.png, report_work.png for the report's final-controller section."""
    label = bag_label(bag)
    # 1. Time-space: the all-IDM baseline, then STG at every penetration rate, one speed scale.
    #    (TG's diagrams look the same at this scale, so they are not drawn.)
    panels = [(BASELINE, 0.0, "All-IDM baseline")] + [("STG", p, f"STG, p = {p:g}") for p in RATES]
    ds = [data.get((bag, p, c)) for c, p, _ in panels]
    norm = plt.Normalize(0, max(d["vmax"] for d in ds if d))
    fig, axes = plt.subplots(1, len(panels), figsize=(20, 4.6), facecolor=SURFACE, sharex=True, sharey=True,
                             layout="constrained")
    for ax, (c, p, title), d in zip(axes, panels, ds):
        style(ax)
        ax.set_title(title, loc="left", fontsize=13)
        if d is not None:
            add_trajectories(ax, d, norm, (0.3, 0.8))
        if c != BASELINE:
            note(ax, metrics.get((bag, p, c)))
        ax.set_xlabel("Time (s)")
    axes[0].set_ylabel("Position (km)")
    fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=CMAP), ax=axes.tolist(), pad=0.01, aspect=25,
                 label="Speed (m/s)")
    fig.savefig(out / "report_time_space.png", dpi=150, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)

    # 2. Fundamental diagram per p
    fig, axes = plt.subplots(1, len(RATES), figsize=(16, 4.2), facecolor=SURFACE, sharex=True, sharey=True,
                             layout="constrained")
    base = data.get((bag, 0.0, BASELINE))
    for ax, p in zip(axes, RATES):
        style(ax)
        if base is not None:
            ax.scatter(*base["fd"], s=10, color=COLOR[BASELINE], alpha=0.35, edgecolors="none",
                       label="all-IDM baseline")
        for c in ("TG", "STG"):
            d = data.get((bag, p, c))
            if d is not None:
                ax.scatter(*d["fd"], s=14, color=COLOR[c], alpha=0.75, edgecolors="none", label=c)
        ax.set_title(f"p = {p:g}", fontsize=12)
        ax.set_xlabel("Density (veh/km)")
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
    axes[0].set_ylabel("Flow (veh/h)")
    axes[0].legend(frameon=False, loc="upper right", markerscale=2, fontsize=9)
    fig.savefig(out / "report_fd.png", dpi=150, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)

    # 3. Accumulated work per p: platoon mean with the 10-90 % range of the cars
    fig, axes = plt.subplots(1, len(RATES), figsize=(16, 4.2), facecolor=SURFACE, sharex=True, sharey=True,
                             layout="constrained")
    for ax, p in zip(axes, RATES):
        style(ax)
        if base is not None:
            ax.plot(base["cw_t"], base["cw"][1:].mean(axis=0), color=COLOR[BASELINE], linewidth=1.8,
                    linestyle=(0, (5, 3)), label="all-IDM baseline")
        for c in ("TG", "STG"):
            d = data.get((bag, p, c))
            if d is None:
                continue
            cw = d["cw"][1:]
            lo, hi = np.percentile(cw, [10, 90], axis=0)
            ax.fill_between(d["cw_t"], lo, hi, color=COLOR[c], alpha=0.15, linewidth=0)
            ax.plot(d["cw_t"], cw.mean(axis=0), color=COLOR[c], linewidth=2, label=f"{c} (mean, 10-90 % band)")
        ax.set_title(f"p = {p:g}", fontsize=12)
        ax.set_xlabel("Time (s)")
        ax.set_ylim(bottom=0)
    axes[0].set_ylabel("Accumulated positive work (m²/s²)")
    axes[0].legend(frameon=False, loc="upper left", fontsize=9)
    fig.savefig(out / "report_work.png", dpi=150, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)


def write_tex_table(summary, tau, lam):
    """report/tables/final_tau<tau>.tex: per bag and p, STG and TG work/flow/throughput and STG vs TG."""
    by = {(r["bag"], r["controller"], float(r["penetration_rate"])): r for r in summary}
    bags = list(dict.fromkeys(r["bag"] for r in summary))
    num = lambda r, k, d=2: "--" if r is None or r.get(k) in ("", None) else f"{float(r[k]):.{d}f}"
    sign = lambda s: f"${s[0]}${s[1:]}"    # LaTeX minus/plus sign
    pct = lambda r, k: "--" if r is None or r.get(k) in ("", None) else sign(f"{float(r[k]):+.1f}")
    lines = [r"\begin{table}[htbp]", r"\centering", r"\scriptsize",
             rf"\caption{{The final controller, STG ($\lambda={lam:g}$, braking only), against TG at the same "
             rf"time gap $\tau={tau:g}$\,s, on every drive and penetration rate $p$. Work in m$^2$/s$^3$, flow "
             rf"and throughput in veh/h; $\Delta$: STG relative to TG (\%). The $p=0$ row is the all-IDM "
             rf"baseline. $\dagger$: a TG car came closer than 5\,m (no STG car ever did).}}",
             rf"\label{{tab:final-tau{tau:g}}}", r"\begin{tabular}{llrrrrrrrrr}", r"\toprule",
             r" & & \multicolumn{3}{c}{Work} & \multicolumn{3}{c}{Flow} & \multicolumn{3}{c}{Throughput} \\",
             r"\cmidrule(lr){3-5}\cmidrule(lr){6-8}\cmidrule(lr){9-11}",
             r"Drive & $p$ & STG & TG & $\Delta$ & STG & TG & $\Delta$ & STG & TG & $\Delta$ \\", r"\midrule"]
    for i, bag in enumerate(bags):
        if i:
            lines.append(r"\midrule")
        b = by.get((bag, BASELINE, 0.0))
        lines.append(rf"{bag_label(bag)} & 0 (IDM) & \multicolumn{{2}}{{c}}{{{num(b, 'avg_positive_work', 3)}}} & & "
                     rf"\multicolumn{{2}}{{c}}{{{num(b, 'flow_mean_veh_per_hr', 0)}}} & & "
                     rf"\multicolumn{{2}}{{c}}{{{num(b, 'throughput_veh_per_hr', 0)}}} & \\")
        for p in RATES:
            s, t = by.get((bag, "STG", p)), by.get((bag, "TG", p))
            crash = lambda r: r is not None and r["status"] == "collision"
            mark = r"$^\dagger$" if t is not None and t["status"].startswith("inside") else ""
            tg = lambda k, d: "crash" if crash(t) else num(t, k, d) + mark
            st = lambda k, d: "crash" if crash(s) else num(s, k, d)
            lines.append(rf" & {p:g} & {st('avg_positive_work', 3)} & {tg('avg_positive_work', 3)} & "
                         rf"{pct(s, 'avg_positive_work_vs_TG_pct')} & {st('flow_mean_veh_per_hr', 0)} & "
                         rf"{tg('flow_mean_veh_per_hr', 0)} & {pct(s, 'flow_mean_veh_per_hr_vs_TG_pct')} & "
                         rf"{st('throughput_veh_per_hr', 0)} & {tg('throughput_veh_per_hr', 0)} & "
                         rf"{pct(s, 'throughput_veh_per_hr_vs_TG_pct')} \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    out = ROOT / "report" / "tables" / f"final_tau{tau:g}.tex"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")


def write_rows(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def run_tau(tau, lam, results, bags, jobs, sample):
    stg, tg = configs(tau, lam)
    study = Study(results, [stg, tg], bags)
    out = ROOT / "plots" / "final_model" / f"tau{tau:g}"
    small = ROOT / "results" / "final_model" / f"tau{tau:g}"
    big = results / "final_model" / f"tau{tau:g}"
    out.mkdir(parents=True, exist_ok=True)
    if sweep_data.EXPORT_CSV:
        small.mkdir(parents=True, exist_ok=True)
        big.mkdir(parents=True, exist_ok=True)

    # Summary table
    summary = []
    for bag in bags:
        b = study.metrics(None, bag, 0)
        if b:
            summary.append({**b, "bag": bag, "bag_label": bag_label(bag), "controller": BASELINE, "tau": ""})
        for p in RATES:
            ms = {c.label: study.metrics(c, bag, p) for c in (stg, tg)}
            for c in (stg, tg):
                m = ms[c.label]
                if m is None:
                    continue
                rec = {**m, "bag": bag, "bag_label": bag_label(bag), "controller": c.label, "tau": tau}
                if c is stg and ms["TG"] and m["status"] != "collision" and ms["TG"]["status"] != "collision":
                    for col in ("avg_positive_work", "flow_mean_veh_per_hr", "throughput_veh_per_hr"):
                        rec[f"{col}_vs_TG_pct"] = round(100 * (float(m[col]) / float(ms["TG"][col]) - 1), 2)
                summary.append(rec)
    fields = ["bag", "bag_label", "controller", "tau", "penetration_rate", "status", "avg_positive_work",
              "avg_positive_work_vs_TG_pct", "flow_mean_veh_per_hr", "flow_mean_veh_per_hr_vs_TG_pct",
              "flow_median_veh_per_hr", "flow_p5_veh_per_hr", "flow_p95_veh_per_hr", "density_mean_veh_per_km",
              "throughput_veh_per_hr", "throughput_veh_per_hr_vs_TG_pct", "marker_x_m", "cars_reached_marker",
              "min_gap_smart_m", "min_gap_m", "min_gap_type", "collision", "smart_definition", "seed", "n_vehicles",
              "run"]
    write_csv(small / "summary.csv", summary, fields)
    write_tex_table(summary, tau, lam)

    # Figure data, one run per worker
    jobs_ = []
    metrics = {}
    for bag in bags:
        d = study.run_dir(None, bag, 0)
        if d:
            jobs_.append((bag, 0.0, BASELINE, str(d), tau, big, sample, sweep_data.EXPORT_CSV))
        for p in RATES:
            for c in (stg, tg):
                metrics[(bag, p, c.label)] = study.metrics(c, bag, p)
                d = study.run_dir(c, bag, p)
                if d:
                    jobs_.append((bag, p, c.label, str(d), tau, big, sample, sweep_data.EXPORT_CSV))
    data, fd_all, veh_all = {}, [], []
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        for d in pool.map(extract, jobs_):
            print(f"  tau {tau:g}: {bag_label(d['bag'])}, p = {d['p']:g}, {d['ctrl']}", flush=True)
            fd_all += d.pop("fd_rows")
            veh_all += d.pop("veh_rows")
            data[(d["bag"], d["p"], d["ctrl"])] = d
    write_csv(small / "fd_boxes.csv", fd_all)
    write_csv(small / "work_per_vehicle.csv", veh_all)

    for old in out.glob("*/"):          # per-(bag, p) figure folders of earlier versions
        shutil.rmtree(old)
    for old in out.glob("time_space_*.png"):   # every-bag time-space grids of earlier versions
        old.unlink()
    for c in ("STG", "TG"):
        draw_work(data, metrics, bags, c, tau, out)
    draw_fd(data, bags, tau, out)
    draw_work_mean(data, bags, tau, out)
    if REPORT_BAG in bags:
        draw_report_figures(data, metrics, tau, lam, out)
    print(f"tau {tau:g}: figures in {out.relative_to(ROOT)}, LaTeX table report/tables/final_tau{tau:g}.tex"
          + (f", CSVs in {small.relative_to(ROOT)} and {big}" if sweep_data.EXPORT_CSV else ""))


def main():
    parser = argparse.ArgumentParser(description="Figures and CSVs for the final STG controller vs TG.")
    parser.add_argument("--results-dir", help="runs.csv + runs/ folder (default: $WTJ_RESULTS_DIR or results/)")
    parser.add_argument("--tau", type=float, nargs="+", default=[2.5], help="time gap(s) in s (default 2.5)")
    parser.add_argument("--lam", type=float, default=2.0, help="STG braking-only damping gain (default 2)")
    parser.add_argument("--bag", nargs="+", help="bag names (default: every bag in bags/)")
    parser.add_argument("--plots-only", action="store_true", help="don't run missing simulations")
    parser.add_argument("--jobs", type=int, default=4, help="parallel workers (simulations and figures)")
    parser.add_argument("--csv", action="store_true", help="also write the CSV tables (summary, boxes, per-vehicle "
                                                           "time series)")
    parser.add_argument("--sample", type=float, default=1.0, help="time step of the large CSVs, s (default 1)")
    args = parser.parse_args()
    sweep_data.EXPORT_CSV = args.csv

    results = results_dir(args.results_dir)
    order = list(BAG_LABELS)   # report order: real drive first
    bags = sorted(args.bag or all_bags(), key=lambda b: order.index(b) if b in order else len(order))
    if not args.plots_only:
        cfgs = [c for tau in args.tau for c in configs(tau, args.lam)]
        n = ensure_runs(results, cfgs, bags, jobs=max(args.jobs, 8))
        print(f"{n} simulation(s) run")
    for tau in args.tau:
        run_tau(tau, args.lam, results, bags, args.jobs, args.sample)


if __name__ == "__main__":
    main()
