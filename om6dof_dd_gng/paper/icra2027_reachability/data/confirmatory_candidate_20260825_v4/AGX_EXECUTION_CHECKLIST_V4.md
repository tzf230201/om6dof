# AGX execution checklist — ICRA confirmatory reachability v4

Status: local source/protocol candidate frozen; AGX deployment, build, 60-scene
catalog, and confirmatory outcomes are still pending. Never run these commands in
ROS domain 0, and do not publish to any controller topic.

## 0. Security and safety gates

- Revoke/rotate the GitHub PAT currently embedded in the AGX repository's `origin`
  URL, then replace `origin` with a credential-free URL. Never place the old token
  in logs or the paper bundle.
- Support the arm and keep the workspace clear. This pipeline is preview-only and
  does not start `ros2_control`, `move_group`, a camera, or a controller, but domain
  0 hardware processes must still be left untouched.
- Functional outcomes may run while other processes exist in domain 0. Timing claims
  require a separate controlled run with unrelated high-CPU perception stopped,
  stable clocks/power mode, and recorded temperature/load.

## 1. Transfer and verify the frozen candidate

Transfer `source_snapshot.tar.gz`, `CONFIRMATORY_PROTOCOL_V4.md`, and this checklist
to `/home/kublab/ros2_ws/confirmatory_candidate_20260825_v4/`. Expected hashes:

```text
source_snapshot.tar.gz
  8e5cae23fca7a07421cca0df4f538601d6d65b315a9d201655134b521054b63f
CONFIRMATORY_PROTOCOL_V4.md
  642c61bfba071792ef23e0e23abf55adb1e4b78c5df3bd3f7d6e7d61e2324461
SOURCE_MANIFEST.json (inside the archive; this is source_tree_sha256)
  50398ac54d00f1bcee28a0e54430fa91b69b2e073cc0ddc087b96a9b7d4df23f
```

On AGX:

```bash
cd /home/kublab/ros2_ws/confirmatory_candidate_20260825_v4
sha256sum source_snapshot.tar.gz CONFIRMATORY_PROTOCOL_V4.md
mkdir -p extracted
tar -xzf source_snapshot.tar.gz -C extracted
sha256sum extracted/SOURCE_MANIFEST.json
```

The three values must match exactly. Stop on any mismatch.

## 2. Preserve the current dirty package and deploy without deletion

Create a recoverable archive before overwriting the 23 reachability files:

```bash
cd /home/kublab/ros2_ws
tar -czf om6dof_dd_gng_before_confirmatory_v4_20260825.tar.gz \
  -C /home/kublab/ros2_ws/src/om6dof om6dof_dd_gng
rsync -a \
  /home/kublab/ros2_ws/confirmatory_candidate_20260825_v4/extracted/package_snapshot/om6dof_dd_gng/ \
  /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/
```

Do not add `--delete` to `rsync`. Verify every deployed file against
`extracted/SOURCE_MANIFEST.json` before building.

## 3. Build and test on AGX

```bash
cd /home/kublab/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to om6dof_dd_gng --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+ 2>&1 | tee confirmatory_build_v4_20260825.log
source install/setup.bash
colcon test --packages-select om6dof_dd_gng \
  --event-handlers console_direct+ 2>&1 | tee confirmatory_test_v4_20260825.log
colcon test-result --verbose 2>&1 | tee confirmatory_test_result_v4_20260825.log
```

Acceptance: build exit 0; all C++/Python tests pass; no error/failure/skipped test.
Then record:

```bash
sha256sum \
  build/om6dof_dd_gng/reachability_graph_node \
  confirmatory_build_v4_20260825.log \
  confirmatory_test_v4_20260825.log \
  confirmatory_test_result_v4_20260825.log
```

## 4. Generate the fixed 60-scene catalog in an isolated domain

The launch computes hashes from the exact expanded URDF, SRDF, and resolved YAML;
the generator aborts if any differs from these frozen expected values.

```bash
cd /home/kublab/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=90
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 src/om6dof/om6dof_dd_gng/scripts/generate_reachability_scene_catalog.py \
  --output /home/kublab/ros2_ws/reachability_scene_catalog_icra_v4.json \
  --catalog-id om6dof_icra_scene_catalog_v4 \
  --scene-count 60 \
  --master-key-hex 600025f316f133ef34d1baf8bb9107b3aed500e069247baa4ebb5d6af45ad92f \
  --ros-domain-id 90 \
  --rmw-implementation rmw_fastrtps_cpp \
  --urdf-sha256 daf37611724f4c8efd69b3b470bf505cfce4732353ea985ec54fbbb01f6d412d \
  --srdf-sha256 730e590951a205ec639ff20613ce3305e18290f0c9a599130d38ce7c86d3d424 \
  --parameters-sha256 abf60d6f21bbb0e9b77558316dfb746dacf3af37814a349c60308f7dcfd22175 \
  2>&1 | tee /home/kublab/ros2_ws/reachability_scene_catalog_icra_v4_generation.log
```

Record the catalog and generation-log SHA-256. Audit must confirm:

- 60 scenes = 30 base trajectories × point/segment;
- 10 scenes in each difficulty × obstacle-kind cell;
- identical start/target/hit data within each two-kind pair;
- every clear/block/detour capsule and exact oracle flag is true;
- generator binary hash equals the benchmark node binary hash.

Do not regenerate after inspecting confirmatory outcomes. If generation or audit
fails, fix the pipeline and declare a new protocol version before collecting outcomes.

## 5. Dry-run the exact confirmatory schedule

Replace `<CATALOG_SHA256>` with the recorded digest. This must report 60 streams,
60 scenes, 30 bases, 180 graph builds, and 21,600 queries.

```bash
python3 src/om6dof/om6dof_dd_gng/scripts/reachability_multiscene_benchmark.py \
  --catalog /home/kublab/ros2_ws/reachability_scene_catalog_icra_v4.json \
  --protocol-id icra_confirmatory_v4 \
  --methods gng,guarded_gng,halton_prm \
  --stream-list 100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159 \
  --sample-count 800 --halton-start-index 17 --gng-guard-fraction 0.75 \
  --domain-base 20 --domain-pool-size 80 \
  --rmw-implementation rmw_fastrtps_cpp \
  --source-tree-sha256 50398ac54d00f1bcee28a0e54430fa91b69b2e073cc0ddc087b96a9b7d4df23f \
  --expected-catalog-sha256 <CATALOG_SHA256> \
  --catalog-generation-log /home/kublab/ros2_ws/reachability_scene_catalog_icra_v4_generation.log \
  --source-snapshot /home/kublab/ros2_ws/confirmatory_candidate_20260825_v4/source_snapshot.tar.gz \
  --protocol-document /home/kublab/ros2_ws/confirmatory_candidate_20260825_v4/CONFIRMATORY_PROTOCOL_V4.md \
  --analyzer-script /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/scripts/analyze_reachability_multiscene.py \
  --output-dir /home/kublab/ros2_ws/reachability_icra_confirmatory_v4 \
  --dry-run
```

## 6. Execute once, then analyze with the bundled analyzer

Run the same command without `--dry-run`. Use a fresh, empty output directory.
Do not use `--resume` to repair a corrupt/partial run; preserve the failed directory
and restart into a new directory. The runner checks domain cleanliness, runtime model
hashes, graph composition, method correlation, endpoint counts, exact validation,
and frozen-input hashes before accepting rows.

After exit 0:

```bash
python3 \
  /home/kublab/ros2_ws/reachability_icra_confirmatory_v4/frozen_inputs/analyze_reachability_multiscene.py \
  --input-dir /home/kublab/ros2_ws/reachability_icra_confirmatory_v4 \
  --output-dir /home/kublab/ros2_ws/reachability_icra_confirmatory_v4/analysis \
  --study-kind confirmatory \
  --bootstrap-repetitions 50000 \
  --permutation-repetitions 100000 \
  --bootstrap-seed 20260824 \
  --timing-context functional-run-not-controlled-timing
```

The analyzer must say `audit=PASS`, `study_kind=confirmatory`, and verify all
21,600 rows plus all frozen artifact hashes. Only then may Table II/III in the paper
be updated. The primary family contains exactly two dynamic exact-valid risk-
difference contrasts with Holm correction.
