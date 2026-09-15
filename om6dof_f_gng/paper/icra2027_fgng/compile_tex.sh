#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
cp template/IEEEtran.bst IEEEtran.bst
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass1.log
bibtex main > build_bibtex.log
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass2.log
pdflatex -interaction=nonstopmode -halt-on-error main.tex > build_pass3.log
