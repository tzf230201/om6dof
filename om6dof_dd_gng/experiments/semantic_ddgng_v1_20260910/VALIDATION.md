# Environment semantic DD-GNG v1 — deployment and validation

Implementation is installed in the AGX package:

`/home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng`

The user explicitly approved source transfer, backup, installation and build/test
via `agx_lan`. No camera runtime, controller, hardware interface, or robot motion
was started by this correction. No Git commit/push or credential change was made.

## Verified results

- ROS 2 Humble package build passed on AGX (48.2 s package build).
- All 12 CTest targets passed: **128 leaf test cases**, zero failures/errors/skips.
  `colcon test-result` reports 140 because it also counts the 12 CTest wrappers.
- 30 new cases: 13 density-core and 17 semantic-attention tests.
- Separate AGX AddressSanitizer + UndefinedBehaviorSanitizer run passed all
  30 new cases, with leak detection enabled and no sanitizer diagnostics.
- Local Windows GCC 15.2.0 C++17 build passed 48 core/camera/reachability tests
  using `-D_GLIBCXX_ASSERTIONS -Wall -Wextra -Wpedantic -Werror`.
- SHA-256 preimage checks passed before installation. All ten installed files
  match `candidate.sha256`. The installed topology executable resolves to the
  new build and contains `semantic_dd_gng_v1` status/metrics identifiers.
- Vendored `ddgng.hpp`, `reachability_graph.hpp` and
  `reachability_graph_node.cpp` remain byte-identical to the pre-edit copies.
  Reachability sections of both YAML configurations remain unchanged.

Five-seed synthetic density regressions use equal point/update budgets for
attention and neutral same-core controls. Attention switches left to right after
capacity is reached. Both S=3 (default learning/insertion/aging, capacity64) and
S=20 stress cases passed directional density-allocation assertions. These are
implementation regressions, not camera evaluations or Jetson planning benchmarks.

## Evidence locations

On AGX, relative to the package:

- `docs/semantic_ddgng_v1.md`: algorithm, parameters, provenance and limitations.
- `experiments/semantic_ddgng_v1_20260910/build.log`
- `experiments/semantic_ddgng_v1_20260910/test.log`
- `experiments/semantic_ddgng_v1_20260910/test_result.log`
- `experiments/semantic_ddgng_v1_20260910/package_test_results/`: 12 XML files.
- `experiments/semantic_ddgng_v1_20260910/sanitized_tests.xml`
- `experiments/semantic_ddgng_v1_20260910/candidate.sha256`
- `experiments/semantic_ddgng_v1_20260910/original.sha256`
- `experiments/semantic_ddgng_v1_20260910/protected_after.sha256`
- `experiments/semantic_ddgng_v1_20260910/before_environment_correction.tar.gz`:
  backup of the five replaced existing files; new files are listed separately
  in `changed_files.txt`. Restore only those scoped files if a rollback is needed.

The directory date is a task label. XML retains the AGX host's 2026-09-11 test
timestamps; wall-clock synchronization with the desktop was not verified.

## Remaining validation boundary

- Live RGB-D/YOLO smoke test and end-to-end physical performance are NOT run.
- This is a **semantic-attention adaptation** of DD-GNG strength rules, not a
  bit-exact reproduction of the original terrain-attention implementation.
- Source-image boxes still associate against current depth under bounded pose
  difference, not exact synchronized source-frame geometry.
- Existing semantic display evidence can reuse a cached result; it is not an
  independent-inference confidence. Density attention is separately deduplicated
  and expires from source receipt time rather than inference completion.
- S^4 emphasis can strongly reduce background-node allocation. Evaluate obstacle
  coverage and false-positive sensitivity before any physical-robot trial.
- Global sampling is not a collision-safety guarantee. Reachability/collision
  validation remains independent; a graph path is not an executed grasp.
- Historical captures, frozen catalogs and benchmark results were not relabeled
  or changed. They do not validate this new environment learner.

## Repeating sanitizer checks (no ROS runtime)

From the AGX package root:

```bash
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I include test/test_dynamic_density_gng.cpp test/test_semantic_density_attention.cpp \
  -lgtest_main -lgtest -pthread \
  -o experiments/semantic_ddgng_v1_20260910/sanitized_tests
ASAN_OPTIONS=detect_leaks=1 experiments/semantic_ddgng_v1_20260910/sanitized_tests \
  --gtest_output=xml:experiments/semantic_ddgng_v1_20260910/sanitized_tests.xml
```
