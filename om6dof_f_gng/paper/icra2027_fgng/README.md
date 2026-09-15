# WA-FGNG — ICRA 2027 research draft

The manuscript studies **World-Anchored Foveated Growing Neural Gas with Bounded
Multi-View Replay**. It is an anonymous English research draft in the official
IEEE conference template. `main.pdf` is the compiled manuscript; `main.tex` and
`references.bib` are editable sources. No submission has been made. The final draft has 7 pages, 4 main figures, 2 tables, and results from 108 completed experiments (72 primary, 18 additional rotation, 18 pose-jitter).

The current ROS package implementation is in `../../include/` and `../../src/`.
The manuscript's evaluated legacy implementation remains in
`../../../../TopoVLA/native_fgng_world/`; moving the manuscript does not relabel
newer ROS changes as evaluated paper results. The 525 MB benchmark remains in
`../../../../TopoVLA/experiments/fgng_world/`, including public-data provenance,
preparation, raw outputs, metric evaluation, and protocol tests.

## What was actually evaluated

- Real public TUM RGB-D depth frames, with motion-capture poses provided as input.
- Camera-coordinate, world-coordinate-only, and memory/replay representations.
- A matched constant-attention ablation that preserves reallocation gating.
- Native DBL-GNG and FPS comparisons using the same retained memory and replay.
- Disjoint training/evaluation pixels and a causal accumulated depth reference.
- A fixed A–B–A world-target schedule, not learned attention or object recognition.

These are **known-pose representation experiments**. They do not measure SLAM,
robot task success, collision safety, or autonomous manipulation. Public recorded
depth evaluation is not an independent ground-truth surface mesh.

## Rebuild the figures and manuscript

Required: Python3 with NumPy/Matplotlib/Pillow, pdflatex, BibTeX, pdfinfo,
pdftotext. Run from this directory:

```bash
./build.sh
```

By default the figure scripts find the sibling workspace checkout at
`TopoVLA/experiments/fgng_world`. If it is stored elsewhere, set
`FGNG_EXPERIMENT_DIR=/absolute/path/to/fgng_world` before running `build.sh`.

The build reads completed results, renders vector PDF/SVG and high-resolution PNG
figures, generates numerical tables, compiles LaTeX, and checks page count,
undefined references, overflow, anonymity, and unresolved result placeholders.
`verification.json` records these mechanical checks. Scientific interpretation
still requires author review.

Figure sources and selection are in `scripts/` and `figures/provenance.json`.
The architecture is an explicitly labeled schematic. All quantitative and
experimental scene figures are generated from saved arrays; their values and
geometry are not AI-synthesized. `input_views.*` gives additional dataset context.

## Scientific and submission notes

`implementation_audit.md` records the original camera-frame/temporal-support
issues and inherited limitations. `literature_audit.md` records the closest prior
work and source-verified ICRA 2027 requirements. Bottom-K support retention, local
attention, and world-coordinate GNG each have prior art; the paper does not claim
to introduce them individually.

The official deadline is September 15, 2026, as documented in the linked official
CFP. The paper is limited to 8 pages including references and acknowledgments, is
anonymous, and includes the required AI-use disclosure. Human authors must review
claims, citations, attribution, the final experimental scope, and submission
metadata before submission. Authorship is intentionally not inferred from local
account names or repository ownership.
