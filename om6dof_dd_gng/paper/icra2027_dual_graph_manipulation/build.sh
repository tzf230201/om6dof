#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
export MPLCONFIGDIR=/tmp/icra2027_dual_graph_mpl
export TEXINPUTS="./template/:${TEXINPUTS:-}"
export BSTINPUTS="./template/:${BSTINPUTS:-}"
# --refresh reads original archived files; the portable bundle already contains
# frozen figures/metrics and can compile without the complete robot repository.
if [[ "${1:-}" == "--refresh" ]]; then
  python3 scripts/analyze_evidence.py
  if [[ -f scripts/check_witness_frames.py ]]; then
    python3 scripts/check_witness_frames.py
  fi
fi
python3 scripts/draw_schematics.py
pdflatex -no-shell-escape -interaction=nonstopmode -halt-on-error main.tex > build-pass1.txt
bibtex main > build-bibtex.txt
pdflatex -no-shell-escape -interaction=nonstopmode -halt-on-error main.tex > build-pass2.txt
pdflatex -no-shell-escape -interaction=nonstopmode -halt-on-error main.tex > build-pass3.txt
python3 scripts/verify_manuscript.py
printf '%s\n' 'Compiled main.pdf. Scientific results remain preliminary; see README.md.'
