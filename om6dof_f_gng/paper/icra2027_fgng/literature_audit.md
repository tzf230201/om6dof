# Literature and submission audit — 10 September 2026

This is a targeted primary-source audit, not an exhaustive systematic review. The current defensible direction is **an empirical study of bounded, persistent world-frame support for a foveated GNG representation under camera motion and attention changes**. Novelty has not been established for the individual ingredients. No first-of-its-kind claim is supported.

## Closest prior work and consequences

| Citation key | Verified contribution / overlap | Consequence for this paper |
|---|---|---|
| `fritzke1995gng` | GNG already grows a competitive graph with error-driven insertion and obsolete-edge removal. | Credit the foundational learning and graph machinery. |
| `fritzke1997utility` | GNG with utility handles rapidly changing distributions by removing unhelpful units and inserting units elsewhere. | Node redistribution and nonstationary adaptation are established ideas. |
| `frezzabuet2008gngt` | GNG-T controls vector-quantization accuracy while following abrupt and continuous distribution changes. | Accuracy-controlled growth/deletion and stability–plasticity management are prior art. |
| `toda2019roi` | ROI-GNG changes accumulated-error discounts for selected properties and utility discounts for total density; the latter reacts to a processing-time threshold. | Both attention allocation and runtime-aware total density predate F-GNG. A maximum-node claim does not alone establish a substantive novelty difference. |
| `saputra2019ddgng`, `saputra2023attention` | DD-GNG changes regional graph density for affordance processing; subsequent work integrates dynamic attention with locomotion. | Do not claim first attention-based topological robot perception or infer locomotion/manipulation gains from offline reconstruction. |
| `toda2022gngdt` | GNG-DT selects winners in position space while maintaining topologies for several properties. The paper reports static RGB-D experiments and identifies dynamic data as future work. | Spatial topology separated from semantic/color features is established. “Multi-viewpoint” in this paper also means different property-dependent clusterings, not solely moving-camera fusion. |
| `ardilla2023msbl` | Extends multi-scale batch GNG with sample-based insertion for faster adaptation to dynamic point distributions. | Batch learning and responsiveness to new data require proper attribution. |
| `siow2024dbl` | Distributed initialization, batch calculations, simultaneous growth, and edge-strength pruning; the conclusion explicitly reports delayed relocation when the maximum node count is reached. | The capacity-saturated attention-switch experiment addresses a stated limitation, but improved results must be measured rather than presumed. Official implementation: [DBL-GNG](https://github.com/CornerSiow/DBL-GNG). |
| `siow2024multicamera` | Fast MS-DBL-GNG extracts features from several RGB-D views; RANSAC estimates extrinsics and transforms point clouds into world coordinates. | World-coordinate/multi-camera GNG mapping is already published. Supplied-pose online support retention is a narrower question than their calibration pipeline. |
| `wang2023dynamic` | Task-dependent particle resolution is selected within graph-based dynamics and MPC for object-pile manipulation. | Adaptive-resolution task representations have strong robotics precedent. This is motivation, not an implemented baseline for a pure mapping experiment. |
| `hornung2013octomap`, `niessner2013hashing`, `whelan2015elastic` | Established probabilistic occupancy, sparse volumetric fusion, and persistent surfel mapping respectively. | World transforms and persistent geometric memory are standard mapping techniques. Do not compare an observed-surface graph to free-space maps as though they provide equivalent guarantees. |

Primary sources for the table are linked below; `references.bib` provides verified bibliographic entries. The first GNG paper is catalogued under NIPS 1994, while the printed volume is conventionally cited as 1995. Fritzke's ICANN paper is 1997 despite the publisher's later online-release timestamp.

## What may distinguish the implemented system

A concrete combination may be useful: (1) metric camera-to-world conversion from supplied poses, (2) explicitly bounded voxel support that survives loss of visibility, (3) a declared replay policy and total per-update sample limit, and (4) attention-modulated graph allocation. Its contribution should be presented as a reproducible design and an evaluated trade-off. Merely adding a transformation and a buffer is a plausible engineering improvement, but insufficient evidence by itself for a strong ICRA novelty argument.

Replay also changes the data distribution: current-view observations and old surface samples have different sampling frequency, visibility, and noise. Uniform voxel replay approximates occupied-voxel support, not uniform image sampling or exact surface-area sampling. State the weighting objective and distinguish sensor stride correction from world-memory replay weighting. A nonzero background attention floor provides a bias, not a proof that all obstacles remain represented.

## Experiments that isolate the question

- Include camera-frame current-view F-GNG, world-frame current-view F-GNG, and world-frame F-GNG with memory; otherwise camera-motion compensation and replay benefits are confounded.
- Include uniform attention with the same memory and sample budget. This establishes whether gains follow from attention, stored geometry, or both.
- Include a simple deterministic voxel/representative-point baseline operating on the same support. If it outperforms the graph for geometry, report that; graph edges need their own utility metric.
- Declare both graph-node budget **N** and support-memory budget **M**. Equal **N** is not equal total memory. Report update time and all retained data structures.
- Repeated seeds measure stochastic sensitivity, not independent scene generalization. Report per-sequence values and scene-level uncertainty; do not treat correlated frames as independent trials.
- If evaluation uses accumulated/fused noisy depth, call it an **observed-surface reference**. It is not a ground-truth mesh. Keep held-out depth frames or points out of the support used to construct the tested graph.
- Ground-truth camera poses supplied to the method make this a known-pose representation experiment. Do not report it as a SLAM or camera-tracking result. Pose-noise perturbations are useful stress tests, but are not estimated-odometry validation.
- A nearest-node geometry metric does not establish topological correctness. Additional evidence may include long-edge violations, connected components relative to known object labels, surface-supported edge fractions, and stability under revisits.
- Static-scene persistence can create ghosts for moved objects. A buffer with aging or eviction is not a visibility-consistent occupancy mapper. State whether dynamic-scene removal is unimplemented.

## Dataset check: TUM RGB-D

The [official dataset page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) supplies RGB-D sequences and motion-capture camera trajectories; data are CC BY 4.0 unless otherwise noted. It is practical for a short known-pose study. This dataset was designed primarily to evaluate trajectory estimation, so a geometry/revisit protocol built on it is a new experimental protocol rather than an official TUM reconstruction leaderboard score.

The [official format specification](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats) states that 16-bit PNG depth is divided by 5000, with zero marking missing depth. Trajectory rows are `timestamp tx ty tz qx qy qz qw` for the RGB optical center in a fixed world frame. Depth is preregistered to RGB; the authors recommend the default ROS intrinsics without additional undistortion for these registered images. Published depth-scale corrections are already applied. Document the exact calibration used; do not apply the correction factor twice. Associate timestamps conservatively and interpolate poses with a declared maximum time gap.

## ICRA 2027 verified requirements

Source: [official full call for papers](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/), accessed 10 September 2026.

- Deadline: **15 September 2026, “11:59 PST”** as printed. Verify the exact displayed timezone in PaperPlaza before submission; do not silently interpret the site's PST abbreviation as another zone.
- **Eight pages maximum including references and acknowledgments**, double-column PDF. Supplemental text also counts within those eight pages.
- **Double-anonymous review**: omit author names and affiliations in the manuscript; all authors must be entered in PaperPlaza at initial submission.
- AI-generated text, figures, images, or code requires an acknowledgment naming the system, affected sections, and manner of use. AI cannot be an author. Ordinary grammar editing is generally outside that disclosure requirement.
- Video: maximum 20 MB, 180 seconds; at least 480 pixels high and 20 fps; progressive mpeg/mp4/mpg. The remaining video window is **17–22 September 2026**. Uploads are closed 10–16 September.
- The page says no planned deadline extension. Linked material is not required reading for reviewers.

The official page has unrelated stale ICRA 2025 workshop/registration text near its bottom; the dedicated 2027 paper-preparation and submission sections above are the relevant instructions. This audit does not claim acceptance or certify the final manuscript's compliance.

Suggested disclosure to adapt to the actual final artifacts, with anonymous wording:

> OpenAI Codex assisted with algorithm implementation, experiment scripts, literature organization, manuscript drafting, and figure-generation code. All reported numerical results were produced by executing the accompanying experiment code. The human authors must verify the final implementation, citations, results, and claims before submission.

The last sentence is a responsibility statement for the current draft, not a claim that author verification has already happened. In a submitted version, list actual manuscript sections and remove the prospective wording only after the human authors complete that verification. Do not list an unverified model version.

## Template provenance

[PaperCept's official LaTeX guidance](https://ras.papercept.net/conferences/support/tex.php) links the [ieeeconf package](https://ras.papercept.net/conferences/support/files/ieeeconf.zip). Downloaded into `template/` on 10 September 2026. The class matches the existing workspace copy byte for byte; SHA-256: `4befef671c2a996889d325f5170d3387bf42aac9a37dcaa93724ad49816e4ec2`. Use `\documentclass[letterpaper,10pt,conference]{ieeeconf}` and `\overrideIEEEmargins`. The final PDF still needs visual, page-count, font-embedding, and PaperPlaza checks.

## Primary-source links

1. [Fritzke, original GNG, NIPS](https://proceedings.neurips.cc/paper/1994/hash/d56b9fc4b0f1be8871f5e1c40c0067e7-Abstract.html)
2. [Fritzke, GNG utility, ICANN 1997](https://link.springer.com/chapter/10.1007/BFb0020222)
3. [Frezza-Buet, GNG-T, Neurocomputing 2008](https://www.sciencedirect.com/science/article/pii/S0925231208000544)
4. [Toda et al., ROI-GNG, author manuscript, ICIRA 2019](https://ousar.lib.okayama-u.ac.jp/files/public/6/60825/20201027170359920033/fulletxt.pdf)
5. [Saputra et al., DD-GNG, IROS 2019](https://doi.org/10.1109/IROS40897.2019.8968003)
6. [Saputra et al., dynamic-attention locomotion, Machines 2023](https://www.mdpi.com/2075-1702/11/6/619)
7. [Toda et al., GNG-DT, Applied Sciences 2022](https://www.mdpi.com/2076-3417/12/3/1705)
8. [Ardilla et al., dynamic MS-BL-GNG, IJAT 2023](https://www.fujipress.jp/ijat/au/ijate001700030206/)
9. [Siow et al., DBL-GNG, Mathematics 2024](https://www.mdpi.com/2227-7390/12/12/1909)
10. [Siow et al., multi-camera GNG mapping, Biomimetics 2024](https://www.mdpi.com/2313-7673/9/9/560)
11. [Wang et al., dynamic-resolution manipulation, RSS 2023](https://www.roboticsproceedings.org/rss19/p047.html)
12. [Hornung et al., OctoMap, official project](https://octomap.github.io/)
13. [Niessner et al., voxel hashing, author manuscript](https://graphics.stanford.edu/~niessner/papers/2013/4hashing/niessner2013hashing.pdf)
14. [Whelan et al., ElasticFusion, RSS 2015](https://www.roboticsproceedings.org/rss11/p01.html)

Bibliographic metadata only for `saputra2019ddgng` and `tunnermann2019saliency` was cross-checked against publisher-deposited DOI records and author publication lists; no full-text-specific claims are made for those entries in this audit. Publisher abstracts/full texts or author manuscripts were inspected for the substantive comparisons above. `reference_metadata.json` records DOI-API responses, including any rate-limited requests; missing API responses are supplemented by the primary-source pages linked here.

## Added audit: bottom-k support memory

The implemented memory selects the smallest fixed hash ranks of voxel keys and stores one representative nearest each retained voxel's center. This belongs to established **bottom-k sampling**, not a new sampling algorithm. [Thorup's author manuscript](https://arxiv.org/abs/1303.5479), published at STOC 2013 ([author institutional record](https://researchprofiles.ku.dk/en/publications/bottom-k-and-priority-sampling-set-similarity-and-subset-sums-wit/)), explicitly states the merge identity `S_k(A ∪ B) = S_k(S_k(A) ∪ S_k(B))`. Cite `thorup2013bottomk` for that foundation.

A short proposition may characterize this implementation: for a fixed camera-pose assignment, voxel grid, capacity M, and total ordering `(hash(key), key)`, an append-only stream retains the M smallest distinct observed voxel keys regardless of stream permutation or repeated identical observations. If representatives are selected by a total ordering such as `(squared distance to voxel center, x, y, z)`, their final values are also invariant. Proof: discarded keys already have M smaller observed keys; adding more keys can never improve their rank. The per-voxel representative is a deterministic minimum.

This is a conditional correctness property of the **support buffer**, not a theorem about the learned graph, noisy geometry, unbiased estimation, or dynamic-object removal. A first-arrival tie-break for equally distant points invalidates representative permutation invariance. Learned graph trajectories, attention schedules, update counts, timestamps, and pose changes can still depend on observation order. Deterministic hashing alone does not warrant claiming statistically uniform sampling without an explicit probabilistic assumption.
