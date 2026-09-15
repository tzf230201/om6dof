#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export MPLCONFIGDIR="${TMPDIR:-/tmp}/fgng-matplotlib"
mkdir -p "$MPLCONFIGDIR"
python3 scripts/draw_architecture.py
python3 scripts/make_figures.py
python3 scripts/make_results.py
cp template/IEEEtran.bst IEEEtran.bst
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass1.log
bibtex main > build_bibtex.log
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass2.log
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass3.log
python3 scripts/check_paper.py
python3 scripts/package_source.py
