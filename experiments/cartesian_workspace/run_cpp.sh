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
wrapper_args=("$@")
model=installed
scan_args=()
slice_step_mm=50
while [[ $# -gt 0 ]]; do
  scan_arg="$1"
  shift
  case "$scan_arg" in
    --model)
      if [[ $# -eq 0 ]]; then echo "Missing value for --model (installed, v1, or v2)." >&2; exit 1; fi
      model="$1"
      shift
      case "$model" in installed|v1|v2) ;; *) echo "Unknown model: $model (use installed, v1, or v2)." >&2; exit 1 ;; esac
      ;;
    --spacing-mm)
      if [[ $# -eq 0 ]]; then echo "Missing value for --spacing-mm." >&2; exit 1; fi
      slice_step_mm="$1"
      scan_args+=("$scan_arg" "$1")
      shift
      ;;
    --output|--urdf|--probe|--self-test)
      echo "Use the executable directly for $scan_arg; the wrapper takes the output directory as its first argument." >&2
      exit 1 ;;
    *) scan_args+=("$scan_arg") ;;
  esac
done
if [[ -e "$output_dir" ]]; then
  echo "Output already exists: $output_dir. Choose a new directory." >&2
  exit 1
fi
output_dir="$(realpath -m -- "$output_dir")"
build_dir=/tmp/om6dof-workspace-cpp-build
cmake -S "$experiment_dir/cpp" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j2
model_dir="$(mktemp -d /tmp/om6dof-workspace-model.XXXXXX)"
model_name=om6dof
if [[ "$model" == installed ]]; then
  # Backward-compatible default: follow the currently sourced ROS overlay.
  description_dir="$(ros2 pkg prefix om6dof_description)/share/om6dof_description"
else
  # Explicit versions always use this checkout, including xacro includes and
  # payload YAML. A temporary ament index prevents stale installed dependencies
  # from being silently mixed with the requested source model. No ROS node runs.
  description_dir="$(cd -- "$experiment_dir/../../om6dof_description" && pwd)"
  model_prefix="$model_dir/source-prefix"
  mkdir -p "$model_prefix/share/ament_index/resource_index/packages"
  ln -s "$description_dir/package.xml" "$model_prefix/share/ament_index/resource_index/packages/om6dof_description"
  ln -s "$description_dir" "$model_prefix/share/om6dof_description"
  export AMENT_PREFIX_PATH="$model_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  if [[ "$model" == v2 ]]; then model_name=om6dof_v2; fi
fi
model_source="$description_dir/urdf/$model_name.urdf.xacro"
payload_source="$description_dir/config/payload.yaml"
if [[ "$model" == v2 ]]; then payload_source="$description_dir/config/payload_d435.yaml"; fi
echo "Model: $model ($model_source)"
xacro "$model_source" -o "$model_dir/model.urdf"
"$build_dir/workspace_scan_cpp" --urdf "$model_dir/model.urdf" --output "$output_dir" "${scan_args[@]}"
# Numerical parameters are recorded by the scanner in summary.json. Keep the
# exact wrapper invocation and input/binary hashes alongside the URDF snapshot.
printf '#!/usr/bin/env bash\n' > "$output_dir/command.sh"
printf '%q ' bash "$experiment_dir/run_cpp.sh" "$output_dir" "${wrapper_args[@]}" >> "$output_dir/command.sh"
printf '\n' >> "$output_dir/command.sh"
sha256sum "$output_dir/model.urdf" "$model_source" \
  "$description_dir/urdf/materials.xacro" "$payload_source" \
  "$experiment_dir/cpp/scan.cpp" "$build_dir/workspace_scan_cpp" \
  > "$output_dir/SHA256SUMS"
"$build_dir/workspace_report_cpp" --input "$output_dir" --slice-step-mm "$slice_step_mm"
echo "Done. Open: $output_dir/index.html"
echo "Analysis: $output_dir/analysis.md"
