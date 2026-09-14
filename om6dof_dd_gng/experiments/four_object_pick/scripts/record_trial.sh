#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${1:-}"
if [[ -z "$RUN_ID" || ! "$RUN_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
  printf 'Usage: %s RUN_ID\n' "$0" >&2
  exit 2
fi
record_duration_sec="${RECORD_DURATION_SEC:-0}"
if [[ ! "$record_duration_sec" =~ ^[0-9]+$ ]]; then
  printf 'RECORD_DURATION_SEC must be a non-negative integer.\n' >&2
  exit 2
fi
if ! awk -F, -v run_id="$RUN_ID" \
  'NR > 1 && $3 == run_id { found=1 } END { exit !found }' \
  "$ROOT_DIR/trial_manifest.csv"; then
  printf 'RUN_ID is not present in the frozen manifest: %s\n' "$RUN_ID" >&2
  exit 2
fi

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

DATA_ROOT="${EXPERIMENT_DATA_ROOT:-$ROOT_DIR/data}"
RUN_DIR="$DATA_ROOT/$RUN_ID"
if [[ -e "$RUN_DIR" ]]; then
  printf 'Refusing to overwrite existing run: %s\n' "$RUN_DIR" >&2
  exit 3
fi

preflight_output=""
preflight_skipped=0
preflight_report_tmp=""
if [[ "${SKIP_PREFLIGHT:-0}" != "1" ]]; then
  preflight_report_tmp="$(mktemp "${TMPDIR:-/tmp}/om6dof_scene_readiness.XXXXXX.json")"
  set +e
  preflight_output="$(SCENE_READINESS_REPORT_JSON="$preflight_report_tmp" \
    "$ROOT_DIR/scripts/preflight.sh" 2>&1)"
  preflight_rc=$?
  set -e
  printf '%s\n' "$preflight_output"
  if [[ "$preflight_rc" -ne 0 ]]; then
    rm -f -- "$preflight_report_tmp"
    printf 'Preflight failed; no run directory was created.\n' >&2
    exit "$preflight_rc"
  fi
else
  preflight_skipped=1
fi
mkdir -p "$RUN_DIR/logs"
early_cleanup() {
  local code="$1"
  trap - EXIT INT TERM
  printf 'initialization_failed_or_interrupted=%s\n' "$code" >>"$RUN_DIR/run_metadata.txt" 2>/dev/null || true
  printf 'incomplete\n' >"$RUN_DIR/run_incomplete.flag" 2>/dev/null || true
  exit "$code"
}
trap 'early_cleanup $?' EXIT
trap 'early_cleanup 130' INT
trap 'early_cleanup 143' TERM
if [[ "$preflight_skipped" == "1" ]]; then
  printf 'SKIPPED by SKIP_PREFLIGHT=1\n' >"$RUN_DIR/preflight.txt"
else
  printf '%s\n' "$preflight_output" >"$RUN_DIR/preflight.txt"
  if [[ -s "$preflight_report_tmp" ]]; then
    cp "$preflight_report_tmp" "$RUN_DIR/scene_readiness.json"
  fi
  rm -f -- "$preflight_report_tmp"
fi

{
  printf 'run_id=%s\n' "$RUN_ID"
  printf 'started_utc=%s\n' "$(date --utc --iso-8601=ns)"
  printf 'preflight_skipped=%s\n' "$preflight_skipped"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'kernel=%s\n' "$(uname -a)"
  printf 'git_head=%s\n' "$(git -C "$HOME/ros2_ws/src/om6dof" rev-parse HEAD 2>/dev/null || true)"
  printf 'git_status_begin\n'
  git -C "$HOME/ros2_ws/src/om6dof" status --short 2>/dev/null || true
  printf 'git_status_end\n'
  printf 'nvpmodel_begin\n'
  nvpmodel -q 2>&1 || true
  printf 'nvpmodel_end\n'
  printf 'jetson_clocks_begin\n'
  jetson_clocks --show 2>&1 || true
  printf 'jetson_clocks_end\n'
  printf 'camera_begin\n'
  rs-enumerate-devices -s 2>&1 || true
  printf 'camera_end\n'
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
  printf 'rmw=%s\n' "${RMW_IMPLEMENTATION:-default}"
  printf 'fastdds_builtin_transports=%s\n' "${FASTDDS_BUILTIN_TRANSPORTS:-default}"
  printf 'ros_domain_id=%s\n' "${ROS_DOMAIN_ID:-default}"
} >"$RUN_DIR/run_metadata.txt"
cp "$ROOT_DIR/experiment.yaml" "$RUN_DIR/experiment.yaml"
cp "$ROOT_DIR/trial_manifest.csv" "$RUN_DIR/trial_manifest.csv"
cp "$ROOT_DIR/config/four_object_topo_gng_v2.yaml" "$RUN_DIR/four_object_topo_gng_v2.yaml"
git -C "$HOME/ros2_ws/src/om6dof" diff -- om6dof_dd_gng >"$RUN_DIR/git_diff_om6dof_dd_gng.patch" 2>/dev/null || true
git -C "$HOME/ros2_ws/src/om6dof" status --short >"$RUN_DIR/git_status.txt" 2>/dev/null || true
find "$HOME/ros2_ws/src/om6dof/om6dof_dd_gng" \
  -path '*/experiments/four_object_pick/data' -prune -o \
  -type f -print0 | sort -z | xargs -0 sha256sum >"$RUN_DIR/source_tree_sha256.txt" 2>/dev/null || true
sha256sum "$HOME/.cache/om6dof_perception/yolox_s.onnx" \
  "$HOME/ros2_ws/install/om6dof_dd_gng/lib/om6dof_dd_gng/topo_gng_node" \
  "$HOME/ros2_ws/install/om6dof_dd_gng/lib/om6dof_dd_gng/reachability_graph_node" \
  >"$RUN_DIR/runtime_artifact_sha256.txt" 2>/dev/null || true
timeout 5 ros2 param dump /topo_gng_node >"$RUN_DIR/topo_gng_params.yaml" 2>/dev/null || true
timeout 5 ros2 param dump /reachability_graph_node >"$RUN_DIR/reachability_params.yaml" 2>/dev/null || true
timeout 5 ros2 topic list -t >"$RUN_DIR/topics_at_start.txt" 2>/dev/null || true

pids=()
names=()
start_bg() {
  local name="$1"
  shift
  "$@" >"$RUN_DIR/logs/${name}.stdout.log" 2>"$RUN_DIR/logs/${name}.stderr.log" &
  pids+=("$!")
  names+=("$name")
}

cleanup_done=0
recording_failed=0
cleanup() {
  if [[ "$cleanup_done" == "1" ]]; then
    return
  fi
  cleanup_done=1
  trap - EXIT INT TERM
  for index in "${!pids[@]}"; do
    if [[ "${names[$index]}" == "tegrastats" ]]; then
      kill -TERM "${pids[$index]}" 2>/dev/null || true
    else
      kill -INT "${pids[$index]}" 2>/dev/null || true
    fi
  done
  shutdown_deadline=$((SECONDS + 12))
  while [[ "$SECONDS" -lt "$shutdown_deadline" ]]; do
    alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=$((alive + 1))
      fi
    done
    [[ "$alive" == "0" ]] && break
    sleep 0.25
  done
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  printf 'stopped_utc=%s\n' "$(date --utc --iso-8601=ns)" >>"$RUN_DIR/run_metadata.txt"
  python3 "$ROOT_DIR/scripts/mark_trial_event.py" "$RUN_DIR" recording_stopped --result na || true
  if ! ros2 bag info "$RUN_DIR/rosbag" >"$RUN_DIR/rosbag_info.txt" 2>&1; then
    recording_failed=1
  fi
  for video in outside_camera.mp4 rviz_desktop.mp4; do
    if [[ -e "$RUN_DIR/$video" ]] && command -v gst-discoverer-1.0 >/dev/null; then
      if ! gst-discoverer-1.0 "$RUN_DIR/$video" >"$RUN_DIR/${video%.mp4}_media_info.txt" 2>&1; then
        recording_failed=1
      fi
    fi
  done
  if [[ -s "$RUN_DIR/tegrastats.log" ]]; then
    if ! python3 "$ROOT_DIR/scripts/parse_tegrastats.py" "$RUN_DIR/tegrastats.log"; then
      recording_failed=1
    fi
  fi
  printf 'recorder_failure_detected=%s\n' "$recording_failed" >>"$RUN_DIR/run_metadata.txt"
  rm -f -- "$RUN_DIR/run_complete.flag" "$RUN_DIR/run_incomplete.flag"
  if [[ "$recording_failed" == "0" && -s "$RUN_DIR/rosbag/metadata.yaml" ]]; then
    printf 'complete\n' >"$RUN_DIR/run_complete.flag"
  else
    printf 'incomplete\n' >"$RUN_DIR/run_incomplete.flag"
  fi
  # Generate the summary only after the final integrity flag and metadata are
  # visible, otherwise the JSON would describe a mid-cleanup state.
  if ! python3 "$ROOT_DIR/scripts/summarize_run.py" "$RUN_DIR"; then
    recording_failed=1
    rm -f -- "$RUN_DIR/run_complete.flag"
    printf 'incomplete\n' >"$RUN_DIR/run_incomplete.flag"
    printf 'summary_generation_failed=1\n' >>"$RUN_DIR/run_metadata.txt"
  fi
  (
    cd "$RUN_DIR"
    find . -maxdepth 2 -type f ! -name recorded_files_sha256.txt -print0 \
      | sort -z | xargs -0 sha256sum
  ) >"$RUN_DIR/recorded_files_sha256.txt" 2>/dev/null || true
  printf 'Recording stopped: %s\n' "$RUN_DIR"
}
trap - EXIT INT TERM
trap cleanup EXIT INT TERM

ROS_TOPICS=(
  /tf /tf_static /joint_states /diagnostics
  /om6dof_topo_gng_v2/status
  /om6dof_topo_gng_v2/perception_metrics
  /om6dof_topo_gng_v2/labels
  /om6dof_topo_gng_v2/environment_graph
  /om6dof_topo_gng_v2/environment_graph_data
  /om6dof_topo_gng_v2/robot_graph
  /om6dof_topo_gng_v2/reachability_graph
  /om6dof_topo_gng_v2/reachability_graph_data
  /om6dof_topo_gng_v2/reachability_plan
  /om6dof_topo_gng_v2/reachability_path
  /om6dof_topo_gng_v2/debug_image/compressed
  /om6dof_topo_gng_v2/reachability_training_samples
  /om6dof_topo_gng_v2/reachability_training_samples_cloud
  /dynamixel_hardware_interface/health
  /arm_controller/controller_state
)
start_bg rosbag ros2 bag record --include-hidden-topics -o "$RUN_DIR/rosbag" "${ROS_TOPICS[@]}"
bag_pid="${pids[${#pids[@]}-1]}"

# Do not declare the experiment started merely because the rosbag process is
# alive. DDS discovery is asynchronous; verify that its node exposes the
# critical subscriptions before starting the metric/visual recorders.
BAG_READY_TOPICS=(
  /joint_states
  /om6dof_topo_gng_v2/status
  /om6dof_topo_gng_v2/perception_metrics
  /om6dof_topo_gng_v2/labels
  /om6dof_topo_gng_v2/environment_graph_data
  /om6dof_topo_gng_v2/reachability_graph_data
  /om6dof_topo_gng_v2/reachability_plan
  /om6dof_topo_gng_v2/debug_image/compressed
)
bag_ready=0
bag_info=""
bag_ready_deadline=$((SECONDS + 20))
while [[ "$SECONDS" -lt "$bag_ready_deadline" ]]; do
  if ! kill -0 "$bag_pid" 2>/dev/null; then
    printf 'rosbag exited during DDS discovery; see logs.\n' >&2
    recording_failed=1
    exit 5
  fi
  bag_info="$(timeout 3 ros2 node info /rosbag2_recorder 2>/dev/null || true)"
  bag_ready=1
  for topic in "${BAG_READY_TOPICS[@]}"; do
    if [[ "$bag_info" != *"$topic:"* ]]; then
      bag_ready=0
      break
    fi
  done
  [[ "$bag_ready" == "1" ]] && break
  sleep 0.25
done
printf '%s\n' "$bag_info" >"$RUN_DIR/rosbag_node_info_at_ready.txt"
if [[ "$bag_ready" != "1" ]]; then
  printf 'rosbag did not expose every critical subscription within 20 seconds.\n' >&2
  recording_failed=1
  exit 6
fi

start_bg module_resources python3 "$ROOT_DIR/scripts/module_resource_monitor.py" \
  --output-dir "$RUN_DIR" --period "${RESOURCE_PERIOD_SEC:-0.5}"
start_bg ros_metrics python3 "$ROOT_DIR/scripts/ros_metrics_recorder.py" --output-dir "$RUN_DIR"

if command -v tegrastats >/dev/null; then
  start_bg tegrastats tegrastats --interval 500 --logfile "$RUN_DIR/tegrastats.log"
fi

if [[ -n "${OUTSIDE_CAMERA_DEVICE:-}" ]]; then
  outside_video_size="${OUTSIDE_VIDEO_SIZE:-1920x1080}"
  outside_width="${outside_video_size%x*}"
  outside_height="${outside_video_size#*x}"
  if command -v ffmpeg >/dev/null; then
    start_bg outside_camera ffmpeg -nostdin -y -f v4l2 -framerate 30 \
      -video_size "$outside_video_size" -i "$OUTSIDE_CAMERA_DEVICE" \
      -c:v libx264 -preset veryfast -crf 18 "$RUN_DIR/outside_camera.mp4"
  elif command -v gst-launch-1.0 >/dev/null; then
    start_bg outside_camera gst-launch-1.0 -e \
      v4l2src device="$OUTSIDE_CAMERA_DEVICE" do-timestamp=true ! decodebin ! \
      videoconvert ! videoscale ! videorate ! \
      video/x-raw,width="$outside_width",height="$outside_height",framerate=30/1 ! \
      timeoverlay halignment=left valignment=top ! \
      x264enc speed-preset=superfast tune=zerolatency bitrate=8000 key-int-max=60 ! \
      h264parse ! mp4mux ! filesink location="$RUN_DIR/outside_camera.mp4"
  fi
fi

if [[ "${CAPTURE_DESKTOP:-0}" == "1" ]] && [[ -n "${DISPLAY:-}" ]]; then
  if command -v ffmpeg >/dev/null; then
    start_bg desktop ffmpeg -nostdin -y -f x11grab -framerate 15 \
      -video_size "${DESKTOP_VIDEO_SIZE:-1920x1080}" -i "${DISPLAY}+0,0" \
      -c:v libx264 -preset veryfast -crf 20 "$RUN_DIR/rviz_desktop.mp4"
  elif command -v gst-launch-1.0 >/dev/null; then
    start_bg desktop gst-launch-1.0 -e \
      ximagesrc display-name="$DISPLAY" use-damage=0 show-pointer=true ! \
      videoconvert ! videorate ! video/x-raw,framerate=15/1 ! \
      timeoverlay halignment=left valignment=top ! \
      x264enc speed-preset=superfast tune=zerolatency bitrate=6000 key-int-max=30 ! \
      h264parse ! mp4mux ! filesink location="$RUN_DIR/rviz_desktop.mp4"
  fi
fi

{
  for index in "${!pids[@]}"; do
    printf '%s,%s\n' "${names[$index]}" "${pids[$index]}"
  done
} >"$RUN_DIR/recorder_pids.csv"

startup_ready=0
startup_deadline=$((SECONDS + 20))
while [[ "$SECONDS" -lt "$startup_deadline" ]]; do
  for index in "${!pids[@]}"; do
    if ! kill -0 "${pids[$index]}" 2>/dev/null; then
      printf 'Recorder %s exited during startup; see logs.\n' "${names[$index]}" >&2
      recording_failed=1
      exit 5
    fi
  done
  startup_ready=1
  for required_file in module_resources.csv system_resources.csv reachability_plan.csv perception_metrics.csv; do
    if [[ ! -s "$RUN_DIR/$required_file" ]]; then
      startup_ready=0
      break
    fi
  done
  for required_stream in reachability_graph.csv environment_graph.csv perception_metrics.csv topic_health.csv; do
    if [[ ! -s "$RUN_DIR/$required_stream" ]] \
        || [[ "$(wc -l <"$RUN_DIR/$required_stream")" -lt 2 ]]; then
      startup_ready=0
      break
    fi
  done
  if [[ "$startup_ready" == "1" ]]; then
    for delivered_topic in debug_image joint_states; do
      if ! awk -F, -v topic="$delivered_topic" \
          'NR > 1 && $3 == topic && ($4 + 0) > 0 { found=1 } END { exit !found }' \
          "$RUN_DIR/topic_health.csv"; then
        startup_ready=0
        break
      fi
    done
  fi
  [[ "$startup_ready" == "1" ]] && break
  sleep 0.25
done
if [[ "$startup_ready" != "1" ]]; then
  printf 'Recorder headers and required live traffic were not ready within 20 seconds.\n' >&2
  recording_failed=1
  exit 6
fi
printf 'recorders_ready_utc=%s\n' "$(date --utc --iso-8601=ns)" >>"$RUN_DIR/run_metadata.txt"
python3 "$ROOT_DIR/scripts/mark_trial_event.py" "$RUN_DIR" recording_started --result na

printf 'Recording %s. Press Ctrl-C once to stop cleanly.\n' "$RUN_DIR"
record_deadline=$((SECONDS + record_duration_sec))
while true; do
  for index in "${!pids[@]}"; do
    if ! kill -0 "${pids[$index]}" 2>/dev/null; then
      printf 'Recorder %s exited unexpectedly; stopping the run.\n' "${names[$index]}" >&2
      recording_failed=1
      exit 7
    fi
  done
  if [[ "$record_duration_sec" -gt 0 && "$SECONDS" -ge "$record_deadline" ]]; then
    break
  fi
  sleep 1
done
