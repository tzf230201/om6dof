#!/usr/bin/env bash
# Build and run only the offline experiment. No service or ROS node is started.
set -eo pipefail
experiment_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$experiment_dir/../../../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$workspace_dir/install/setup.bash" ]]; then
  source "$workspace_dir/install/setup.bash"
fi
set -u
output_dir="${1:-/tmp/om6dof-workspace-cpp-$(date +%Y%m%d-%H%M%S)}"
if [[ $# -gt 0 ]]; then shift; fi
slice_step_mm=50
previous_arg=
for scan_arg in "$@"; do
  if [[ "$previous_arg" == --spacing-mm ]]; then slice_step_mm="$scan_arg"; fi
  case "$scan_arg" in
    --output|--urdf|--probe|--self-test)
      echo "Use the executable directly for $scan_arg; the wrapper takes the output directory as its first argument." >&2
      exit 1 ;;
  esac
  previous_arg="$scan_arg"
done
if [[ -e "$output_dir" ]]; then
  echo "Output already exists: $output_dir. Choose a new directory." >&2
  exit 1
fi
build_dir=/tmp/om6dof-workspace-cpp-build
cmake -S "$experiment_dir/cpp" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j2
model_dir="$(mktemp -d /tmp/om6dof-workspace-model.XXXXXX)"
description_dir="$(ros2 pkg prefix om6dof_description)/share/om6dof_description"
xacro "$description_dir/urdf/om6dof.urdf.xacro" -o "$model_dir/model.urdf"
"$build_dir/workspace_scan_cpp" --urdf "$model_dir/model.urdf" --output "$output_dir" "$@"
"$build_dir/workspace_report_cpp" --input "$output_dir" --slice-step-mm "$slice_step_mm"
echo "Done. Open: $output_dir/index.html"
echo "Analysis: $output_dir/analysis.md"
