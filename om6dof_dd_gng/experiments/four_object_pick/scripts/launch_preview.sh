#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

if command -v systemctl >/dev/null \
    && systemctl --user is-active --quiet om6dof-dd-gng.service; then
  printf '%s\n' \
    'The competing om6dof-dd-gng.service is active and will respawn legacy dd_gng_yolo.py.' \
    'Run: systemctl --user stop om6dof-dd-gng.service' \
    'Restore it after the experiment with: systemctl --user start om6dof-dd-gng.service' >&2
  exit 4
fi

conflicts="$(pgrep -af '[o]m6dof_perception.*perception_node|[d]d_gng_yolo\.py|[d]dgng_realsense\.py|[t]opo_gng_node' || true)"
if [[ -n "$conflicts" ]]; then
  printf 'A process already owns or may own the RealSense camera:\n%s\n' "$conflicts" >&2
  printf 'Stop the conflicting perception service/process before launching this preview.\n' >&2
  exit 4
fi

python3 "$ROOT_DIR/scripts/validate_experiment.py"

exec ros2 launch om6dof_dd_gng topo_gng_node.launch.py \
  model_version:=v2 \
  publish_robot_state:=true \
  launch_reachability:=true \
  launch_rviz:="${LAUNCH_RVIZ:-true}" \
  params_file:="$ROOT_DIR/config/four_object_topo_gng_v2.yaml"
