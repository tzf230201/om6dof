# Anonymous ICRA 2027 manuscript source

Main file: main.tex. Compiled paper: main.pdf.

To compile the supplied LaTeX and precomputed vector figures:

```bash
bash compile_tex.sh
```

Requires TeX Live or another LaTeX distribution with pdflatex and BibTeX. The
ieeeconf class and IEEEtran bibliography style are included from official
sources. No dataset download is required to compile this manuscript.

Numerical macros, tables, and figures are generated from measured experiments.
The evaluated executable and large reproduction dataset remain in the sibling
TopoVLA checkout under `native_fgng_world` and `experiments/fgng_world`. The
current ROS package contains newer implementation changes that are not evidence
for the existing numerical claims. This manuscript source archive is not the
full experimental dataset bundle.

The paper is anonymous, includes the required AI-use disclosure, and has not
been submitted. Review scientific claims, authorship, attribution, and conference
submission metadata before uploading. Data-derived figures use the TUM RGB-D
benchmark (Sturm et al., IROS 2012), CC BY 4.0; see the cited dataset source.
