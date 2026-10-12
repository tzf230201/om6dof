#!/usr/bin/env bash
# Run the current working-tree ATC-DT build in an isolated overlay.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"
build_dir="/tmp/om6dof_atcdt_current_build"
install_dir="/tmp/om6dof_atcdt_current_install"
log_dir="/tmp/om6dof_atcdt_current_log"

# ROS Humble setup scripts reference optional unset variables; do not combine
# their sourcing with bash nounset mode.
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "${log_dir}" build --base-paths "${package_dir}" \
  --build-base "${build_dir}" --install-base "${install_dir}" \
  --packages-select om6dof_f_gng --symlink-install
set +u
source "${install_dir}/setup.bash"
set -u
exec ros2 launch om6dof_f_gng atc_dt_d435.launch.py "$@"
