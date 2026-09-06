# ICRA 2027 LaTeX draft

This directory contains the anonymous, double-column LaTeX conversion of the
confirmatory-v4 reachability manuscript.

## Build

Run from this directory:

```text
python figures/generate_method_figures.py
pdflatex root.tex
bibtex root
pdflatex root.tex
pdflatex root.tex
```

The Python step regenerates the two deterministic, publication-ready method
figures (`roadmap_construction.pdf` and `lazy_query_pipeline.pdf`). It requires
Matplotlib and NumPy; the generated PDF figures are vector graphics.

The main source must remain named `root.tex` for Papercept source conversion.

## Template provenance

- Official Papercept package: `ieeeconf.zip`
- Download URL: `https://ras.papercept.net/conferences/support/files/ieeeconf.zip`
- Downloaded package SHA-256: `11D1051D5FE3DAFD1E25BC7A8B66265CBE65DC135E26440CC6661BAEEEB90C76`
- Official bibliography package: `IEEEtranBST.zip`
- Downloaded package SHA-256: `C6C0AA0EF6794EB502223EB6BF0CA6DFC5BE0D8F6D288D06BBC8C9DE3E5287A3`

The ICRA 2027 submission must use US Letter, double-column format, contain no
author identity, and fit within eight total pages including references.
