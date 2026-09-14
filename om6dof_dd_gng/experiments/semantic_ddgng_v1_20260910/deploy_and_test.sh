#!/usr/bin/env bash
set -euo pipefail
task_package=/home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng
task_run="$task_package/experiments/semantic_ddgng_v1_20260910"
test "$(realpath "$task_run")" = "$task_run"
cd "$task_package"
# Refuse to replace source being used by an active topology launch/build.
if pgrep -x topo_gng_node >/dev/null || pgrep -f '^/.*reachability_graph_node( |$)' >/dev/null; then
  echo 'Active topology process: stop and request direction; nothing deployed.' >&2
  exit 2
fi
sha256sum -c "$task_run/original.sha256"
for task_file in include/om6dof_dd_gng/dynamic_density_gng.hpp include/om6dof_dd_gng/semantic_density_attention.hpp test/test_dynamic_density_gng.cpp test/test_semantic_density_attention.cpp docs/semantic_ddgng_v1.md; do
  test ! -e "$task_file" || { echo "New path already exists: $task_file" >&2; exit 3; }
done
test ! -e "$task_run/before_environment_correction.tar.gz"
tar -czf "$task_run/before_environment_correction.tar.gz" src/topo_gng_node.cpp CMakeLists.txt README.md config/topo_gng.yaml config/topo_gng_v2.yaml
(cd "$task_run/candidate" && sha256sum -c "$task_run/candidate.sha256")
while IFS= read -r task_file; do
  test -n "$task_file" || continue
  install -D -m 0644 "$task_run/candidate/$task_file" "$task_package/$task_file"
done < "$task_run/changed_files.txt"
sha256sum include/om6dof_dd_gng/ddgng.hpp src/reachability_graph_node.cpp include/om6dof_dd_gng/reachability_graph.hpp > "$task_run/protected_after.sha256"
cd /home/kublab/ros2_ws
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --packages-select om6dof_dd_gng --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON 2>&1 | tee "$task_run/build.log"
colcon test --packages-select om6dof_dd_gng --event-handlers console_direct+ 2>&1 | tee "$task_run/test.log"
colcon test-result --test-result-base build/om6dof_dd_gng --verbose 2>&1 | tee "$task_run/test_result.log"
cd "$task_package"
sha256sum -c "$task_run/candidate.sha256"
sha256sum -c "$task_run/protected_after.sha256"
