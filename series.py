"""
How the comparison plots (plot_work_sweep.py, compare_flow.py) group runs into
series and panels, and the colors and markers each series gets.

A run's series label is its smart controller with default parameters ("WTJ3"),
or its controller_params from runs.csv when any were changed ("WTJ3 gmin=5").

Panels:
  1. "Controllers": every smart controller at default parameters (WTJ0-WTJ4, TG).
  2. One "<controller> variants" panel per controller that has runs with changed
     parameters: the default line again, plus one line per variant.
Splitting them keeps each panel at three or four series, so the colors stay
distinguishable (checked with the dataviz palette validator).
"""

CONTROLLER_COLORS = {"WTJ0": "#2a78d6", "WTJ1": "#1baf7a", "WTJ2": "#6250d6", "WTJ3": "#008300",
                     "WTJ4": "#eda100", "TG": "#e34948", "STG": "#8f1d1b"}
CONTROLLER_MARKERS = {"WTJ0": "o", "WTJ1": "s", "WTJ2": "v", "WTJ3": "p", "WTJ4": "D", "TG": "^", "STG": "<"}
# Variant lines (variants panel), in order of first appearance; kept
# off the controller colors so a color means one thing across panels
VARIANT_COLORS = ["#e87ba4", "#9a3412", "#0e7490"]
VARIANT_MARKERS = ["P", "h", "X"]
FALLBACK_COLOR = "#7a7974"

# Runs that finished but had a smart car inside the 5 m safe gap (runs.csv
# "unsafe" column) get an open ring around their point, in this ink color
UNSAFE_RING = dict(marker="o", s=330, facecolors="none", edgecolors="#c2410c", linewidths=2.0, zorder=5)
UNSAFE_LABEL = "smart car within 5 m of the car ahead"


def is_unsafe(row):
    """True for a run that finished but had a smart car inside the safe gap (collisions are handled apart)."""
    return bool(row and row.get("unsafe")) and not row.get("collision")


def unsafe_handle():
    from matplotlib.lines import Line2D
    return Line2D([], [], marker="o", linestyle="", markersize=12, markerfacecolor="none",
                  markeredgecolor=UNSAFE_RING["edgecolors"], markeredgewidth=2, label=UNSAFE_LABEL)


def is_secondary(label):
    """
    True for series drawn with the second style (dashed line, hollow marker,
    hatched box): any run with changed parameters ("WTJ3 gmin=5").
    """
    return label != controller_of(label)


def series_label(row):
    """'WTJ3' for default parameters, else the run's controller_params, e.g. 'WTJ3 gmin=5'."""
    params = row.get("controller_params") or "defaults"
    return row["smart_controller"] if params == "defaults" else params


def controller_of(label):
    return label.split(" ", 1)[0]


def panels(labels, only=None):
    """
    labels: every series label present (p > 0 runs). Returns
    [(panel title, [(label, color, marker), ...]), ...].

    With `only` (a list of labels), one panel with just those series, in that
    order. A variant takes its controller's color and marker, and is told apart
    by the second style (is_secondary).
    """
    labels = set(labels)
    if only:
        return [("Selected series", [(lab, CONTROLLER_COLORS.get(controller_of(lab), FALLBACK_COLOR),
                                      CONTROLLER_MARKERS.get(controller_of(lab), "o"))
                                     for lab in only if lab in labels])]
    defaults = [c for c in CONTROLLER_COLORS if c in labels]
    out = [("Controllers (default parameters)",
            [(c, CONTROLLER_COLORS[c], CONTROLLER_MARKERS[c]) for c in defaults])]
    for c in CONTROLLER_COLORS:
        variants = sorted(lab for lab in labels if controller_of(lab) == c and lab != c)
        if not variants:
            continue
        series = [(c, CONTROLLER_COLORS[c], CONTROLLER_MARKERS[c])] if c in labels else []
        for i, lab in enumerate(variants):
            series.append((lab, VARIANT_COLORS[i] if i < len(VARIANT_COLORS) else FALLBACK_COLOR,
                           VARIANT_MARKERS[i] if i < len(VARIANT_MARKERS) else "o"))
        out.append((f"{c} variants", series))
    return [p for p in out if p[1]]
