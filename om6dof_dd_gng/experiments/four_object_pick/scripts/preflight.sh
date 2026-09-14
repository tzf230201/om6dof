#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
failures=0
warnings=0

pass() { printf 'PASS  %s\n' "$1"; }
warn() { printf 'WARN  %s\n' "$1"; warnings=$((warnings + 1)); }
fail() { printf 'FAIL  %s\n' "$1"; failures=$((failures + 1)); }

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  fail "ROS 2 Humble setup not found"
fi
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
else
  fail "workspace install/setup.bash not found; build first"
fi
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

command -v ros2 >/dev/null && pass "ros2 CLI available" || fail "ros2 CLI missing"
command -v python3 >/dev/null && pass "python3 available" || fail "python3 missing"
command -v tegrastats >/dev/null && pass "tegrastats available" || warn "tegrastats missing"
if command -v ffmpeg >/dev/null; then
  pass "ffmpeg available for external/screen video"
elif command -v gst-launch-1.0 >/dev/null; then
  pass "GStreamer available as external/screen video fallback"
else
  warn "neither ffmpeg nor GStreamer is available; external/screen video disabled"
fi
command -v rs-enumerate-devices >/dev/null && pass "RealSense tools available" || warn "rs-enumerate-devices missing"

if command -v systemctl >/dev/null \
    && systemctl --user is-active --quiet om6dof-dd-gng.service; then
  fail "competing om6dof-dd-gng.service is active; stop it before this frozen experiment"
else
  pass "competing DD-GNG web-stream service is inactive"
fi
legacy_yolo_processes="$(pgrep -af '[d]d_gng_yolo\.py|[d]dgng_realsense\.py' || true)"
if [[ -n "$legacy_yolo_processes" ]]; then
  fail "legacy RealSense/Yolo process is active: $legacy_yolo_processes"
else
  pass "no legacy RealSense/Yolo process is active"
fi

if python3 "$ROOT_DIR/scripts/validate_experiment.py"; then
  pass "frozen experiment manifest/config validated"
else
  fail "frozen experiment manifest/config validation failed"
fi

if [[ -n "${OUTSIDE_CAMERA_DEVICE:-}" ]]; then
  [[ -c "$OUTSIDE_CAMERA_DEVICE" ]] \
    && pass "outside camera device available: $OUTSIDE_CAMERA_DEVICE" \
    || fail "outside camera device is not a character device: $OUTSIDE_CAMERA_DEVICE"
else
  warn "OUTSIDE_CAMERA_DEVICE is unset; use a separate UVC camera or record externally"
fi

MODEL="$HOME/.cache/om6dof_perception/yolox_s.onnx"
[[ -s "$MODEL" ]] && pass "YOLOX model present" || fail "YOLOX model missing: $MODEL"

if command -v rs-enumerate-devices >/dev/null; then
  rs-enumerate-devices -s 2>/dev/null | grep -Eq 'D435|D435I' \
    && pass "D435/D435i detected" || fail "D435/D435i not detected"
fi

if command -v ros2 >/dev/null; then
  ros2 pkg prefix om6dof_dd_gng >/dev/null 2>&1 \
    && pass "om6dof_dd_gng installed" || fail "om6dof_dd_gng not in sourced workspace"
  topics="$(timeout 5 ros2 topic list 2>/dev/null || true)"
  for topic in \
    /joint_states \
    /om6dof_topo_gng_v2/status \
    /om6dof_topo_gng_v2/environment_graph_data \
    /om6dof_topo_gng_v2/reachability_graph_data \
    /om6dof_topo_gng_v2/reachability_plan; do
    if ! grep -Fxq "$topic" <<<"$topics"; then
      fail "topic missing: $topic"
      continue
    fi
    topic_info="$(timeout 5 ros2 topic info "$topic" 2>/dev/null || true)"
    if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$topic_info"; then
      pass "topic has publisher: $topic"
    else
      fail "topic has no publisher: $topic"
    fi
  done
  for visual_topic in \
    /om6dof_topo_gng_v2/debug_image/compressed \
    /om6dof_topo_gng_v2/perception_metrics; do
    visual_info="$(timeout 5 ros2 topic info "$visual_topic" 2>/dev/null || true)"
    grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$visual_info" \
      && pass "required instrumentation publisher active: $visual_topic" \
      || fail "required instrumentation publisher missing: $visual_topic"
  done
  yolo_period="$(timeout 5 ros2 param get /topo_gng_node yolo_period_sec 2>/dev/null || true)"
  grep -Eq '0\.5([[:space:]]|$)' <<<"$yolo_period" \
    && pass "YOLO period is 0.5 s (2 Hz)" || fail "could not verify YOLO 2 Hz: $yolo_period"
  target_classes="$(timeout 5 ros2 param get /topo_gng_node target_classes 2>/dev/null || true)"
  grep -q 'bottle,cup,banana,remote' <<<"$target_classes" \
    && pass "four target classes frozen" || fail "target_classes is not frozen to bottle,cup,banana,remote"
  status_sample="$(timeout 5 ros2 topic echo --once --full-length /om6dof_topo_gng_v2/status 2>/dev/null || true)"
  if grep -q '\"state\":\"mapping\"' <<<"$status_sample" && grep -q '\"accepted\":true' <<<"$status_sample"; then
    pass "topology status reports accepted mapping"
  else
    fail "topology status is not an accepted mapping state"
  fi

  scene_readiness_mode="${SCENE_READINESS_MODE:-experiment}"
  scene_readiness_args=(
    --mode "$scene_readiness_mode"
    --min-supporting-nodes "${SCENE_MIN_SUPPORTING_NODES:-3}"
    --stable-duration-sec "${SCENE_STABLE_DURATION_SEC:-1.0}"
    --max-message-gap-sec "${SCENE_MAX_MESSAGE_GAP_SEC:-0.5}"
    --timeout-sec "${SCENE_READINESS_TIMEOUT_SEC:-15.0}"
  )
  if [[ -n "${SCENE_READINESS_REPORT_JSON:-}" ]]; then
    scene_readiness_args+=(--report-json "$SCENE_READINESS_REPORT_JSON")
  fi
  if scene_readiness_output="$(python3 "$ROOT_DIR/scripts/check_scene_labels.py" "${scene_readiness_args[@]}" 2>&1)"; then
    if [[ "$scene_readiness_mode" == "instrumentation" ]]; then
      pass "semantic scene gate explicitly skipped for instrumentation-only smoke"
    else
      pass "all four semantic classes have stable simultaneous DD-GNG support"
    fi
  else
    fail "semantic scene readiness gate failed: $scene_readiness_output"
  fi
fi

available_kb="$(df -Pk "$ROOT_DIR" | awk 'NR==2 {print $4}')"
if [[ "${available_kb:-0}" -ge 5242880 ]]; then
  pass "at least 5 GiB free for recording"
else
  fail "less than 5 GiB free for recording"
fi

if [[ -e /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]]; then
  pass "CPU frequency telemetry available"
else
  warn "CPU frequency telemetry not exposed"
fi

printf '\nSummary: %d failure(s), %d warning(s).\n' "$failures" "$warnings"
if [[ "$failures" -ne 0 ]]; then
  exit 1
fi
