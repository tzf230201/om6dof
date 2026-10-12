#!/usr/bin/env bash
# Run the exact ATC-DT checkpoint without mixing it with the current source tree.
set -euo pipefail

checkpoint_ref="8581627"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git -C "${script_dir}/../.." rev-parse --show-toplevel)"
worktree_dir="/tmp/om6dof_atcdt_checkpoint_${checkpoint_ref}"
build_dir="/tmp/om6dof_atcdt_checkpoint_build_${checkpoint_ref}"
install_dir="/tmp/om6dof_atcdt_checkpoint_install_${checkpoint_ref}"
log_dir="/tmp/om6dof_atcdt_checkpoint_log_${checkpoint_ref}"

if [[ ! -e "${worktree_dir}/.git" ]]; then
  git -C "${repo_root}" worktree add --detach "${worktree_dir}" "${checkpoint_ref}"
fi

# ROS Humble setup scripts reference optional unset variables; do not combine
# their sourcing with bash nounset mode.
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "${log_dir}" build --base-paths "${worktree_dir}/om6dof_f_gng" \
  --build-base "${build_dir}" --install-base "${install_dir}" \
  --packages-select om6dof_f_gng --symlink-install
set +u
source "${install_dir}/setup.bash"
set -u
exec ros2 launch om6dof_f_gng atc_dt_d435.launch.py "$@"
