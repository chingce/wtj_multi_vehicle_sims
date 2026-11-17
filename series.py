"""
How the comparison plots (plot_work_sweep.py, compare_flow.py) group runs into
series and panels, and the colors and markers each series gets.

A run's series label is its smart controller with default parameters ("WTJ3"),
or its controller_params from runs.csv when any were changed ("WTJ3 gmin=5").

Panels:
  1. "Controllers": every smart controller at default parameters (WTJ, WTJ2, WTJ3, WTJ4, TG).
  2. One "<controller> variants" panel per controller that has runs with changed
     parameters: the default line again, plus one line per variant.
Splitting them keeps each panel at three or four series, so the colors stay
distinguishable (checked with the dataviz palette validator).
"""

CONTROLLER_COLORS = {"WTJ": "#2a78d6", "WTJ2": "#1baf7a", "WTJ3": "#eda100", "WTJ4": "#008300", "TG": "#e34948"}
CONTROLLER_MARKERS = {"WTJ": "o", "WTJ2": "s", "WTJ3": "D", "WTJ4": "p", "TG": "^"}
# Variant lines (variants panel), in order of first appearance; kept
# off the controller colors so a color means one thing across panels
VARIANT_COLORS = ["#6250d6", "#e87ba4", "#9a3412"]
VARIANT_MARKERS = ["v", "P", "h"]
FALLBACK_COLOR = "#7a7974"


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
