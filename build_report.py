"""
Assemble the Overleaf-ready report: copies the figures that report/main.tex
uses into report/figures/, checks that every figure and table it references
exists, and zips the folder to report.zip (upload it to Overleaf with
"New Project > Upload Project").

    uv run iteration.py   --results-dir ...     # figures in plots/iteration, tables in report/tables
    uv run final_model.py --results-dir ...     # figures in plots/final_model/tau2.5, report/tables/final_tau2.5.tex
    uv run build_report.py

report/
  main.tex
  tables/*.tex                       written by iteration.py and final_model.py
  figures/<...>/*.png                only the figures main.tex uses, copied from plots/iteration/
                                     and plots/final_model/tau<tau>/
"""

import argparse
import re
import shutil
import zipfile
from pathlib import Path

ROOT = Path(__file__).parent
REPORT = ROOT / "report"


def main():
    parser = argparse.ArgumentParser(description="Copy figures into report/ and zip it for Overleaf.")
    parser.add_argument("--tau", type=float, default=2.5, help="final-model time gap shown in the report (default 2.5)")
    args = parser.parse_args()

    figs = REPORT / "figures"
    if figs.exists():
        shutil.rmtree(figs)
    sources = {figs / "iteration": ROOT / "plots" / "iteration",
               figs / "final_model": ROOT / "plots" / "final_model" / f"tau{args.tau:g}"}

    # Copy only the figures main.tex uses; every figure and table it names must exist
    tex = (REPORT / "main.tex").read_text(encoding="utf-8")
    missing = [f"tables/{t}.tex" for t in re.findall(r"\\input\{tables/([^}]+)\.tex\}", tex)
               if not (REPORT / "tables" / f"{t}.tex").is_file()]
    n = 0
    for name in re.findall(r"\\includegraphics(?:\[[^\]]*\])?\{([^}\\]+)\}", tex):
        hit = next(((dst, src / name) for dst, src in sources.items() if (src / name).is_file()), None)
        if hit is None:
            missing.append(name)
            continue
        out = hit[0] / name
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(hit[1], out)
        n += 1
    if missing:
        raise SystemExit("main.tex refers to missing files (run iteration.py and final_model.py first):\n  "
                         + "\n  ".join(missing))
    print(f"Copied {n} figures to {figs.relative_to(ROOT)}")

    zip_path = ROOT / "report.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(REPORT.rglob("*")):
            if f.is_file() and f.suffix in (".tex", ".png", ".bib", ".sty", ".cls"):
                z.write(f, f.relative_to(REPORT))
    print(f"All references found. Overleaf upload: {zip_path} ({zip_path.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
