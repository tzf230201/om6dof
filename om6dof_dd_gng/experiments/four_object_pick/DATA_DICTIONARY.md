# Data dictionary and measurement boundaries

## Clocks

- `unix_ns`/`received_utc` align files on the AGX host clock.
- `monotonic_ns` is the authoritative clock for elapsed time.
- RealSense device timestamps are recorded only as device-clock metadata; they
  are not treated as Unix/ROS time because hardware synchronization is not
  currently established.

## Process resources

`module_resources.csv` samples Linux `/proc` every 0.5 s by default.

- `rss_mb`: resident pages charged to each matching process; shared pages can
  be counted in more than one process.
- `pss_mb`: proportional set size; the preferred cross-module footprint metric.
- `cpu_percent`: aggregate process CPU, so a multithreaded process can exceed
  100% on a multi-core AGX.
- `read_mb_s`/`write_mb_s`: kernel-accounted storage I/O rate.
- `threads`: sum across all PIDs matched to that logical module.
- `cpu_sample_complete`, `pss_sample_complete`, and `io_sample_complete` state
  whether every owned PID contributed to that measurement. Invalid values are
  blank/excluded, never converted to zero.
- `process_identities_json` records the executable identities behind a module;
  one PID can belong to at most one module. Ambiguous matches are excluded and
  written to `process_match_conflicts.csv`.

DD-GNG and asynchronous YOLO run inside `topo_gng_node`; their memory cannot be
split with `/proc`. Report their combined PSS/RSS, and use
`perception_metrics` for their separate compute times. Allocation rate or heap
churn would require a dedicated allocator profiler and is not inferred from
RSS/PSS.

The Dynamixel hardware interface is a plugin inside `ros2_control_node`.
Accordingly, its process memory is reported as the combined `ros2_control`
module; a separate hardware-interface memory number would be false precision.

`system_resources.csv` records total CPU/load/RAM/swap. `tegrastats.log` is the
source for Jetson GPU utilization, temperature, clocks, memory-controller load,
and board power where exposed by the platform.

`tegrastats_tidy.csv` is the parsed long-form table. Each row retains the
source sample/line, metric, component, value, unit, and parse status.
`tegrastats_summary.json` reports coverage and descriptive statistics per
available component. Missing clocks/rails remain absent; power rails are not
summed because their domains can overlap on Jetson platforms.

The experiment defaults to `rmw_fastrtps_cpp` with
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. This remains interoperable with the active
robot stack while avoiding stale shared-memory lock errors observed during
smoke testing. Both values are copied into `run_metadata.txt`.

## Latency

- Perception stage fields measure host-receipt-to-publication work with a
  monotonic clock. They are not sensor-exposure-to-output latency.
- `planning_time_ms` is the reachability node's internal planning interval.
- `exact_time_ms` is exact MoveIt/FCL validation inside that interval.
- `received_monotonic_ns` is recorder reception time, not a query start time.
  Therefore live-preview rows do not establish end-to-end query latency.
- A GNG-versus-PRM speed claim requires atomic frozen queries with a recorded
  query-publish monotonic timestamp and exactly one terminal plan per query.

## Trial joins and endpoints

`opportunity_id` is the immutable join key between `trial_manifest.csv` and
`operator_events.csv`. Object identity and position are copied from the frozen
manifest rather than typed by the operator. The current live planner still
does not carry this key, so plan publications must not be retrospectively
assigned to an object unless an automated target selector/atomic query created
the association.

Stage-C grasp fields remain `not_run` until the target selector, grasp poses,
executor, collision recheck, and emergency-stop gates are satisfied.

## Readiness, delivery, and completeness

`scene_readiness.json` is produced before a normal run. `semantic_ready=true`
requires `bottle`, `cup`, `banana`, and `remote` to each have at least three
unique labeled node IDs in the same messages for a continuous 1 s. A malformed
payload, support below threshold, or a labels-topic gap above 0.5 s resets the
interval. Instrumentation-only smoke mode records `semantic_ready=null` and
cannot satisfy experimental readiness.

`topic_health.csv` contains startup, periodic, and final shutdown-partial
windows plus cumulative delivered counts. It covers semantic/environment/
planner messages and the actual debug-image and joint-state deliveries.

`run_complete.flag` means recorder and bag shutdown integrity only. The
separate `run_summary.json:completeness.analysis_ready` also requires the
semantic gate, `scene_ready`, all manifest opportunities to have terminal
events, nonempty metric streams, unambiguous resource ownership, and matching
final topic totals. Activity from the legacy `dd_gng_yolo.py`, ordinary
perception process, or a physical pick executor is recorded as contamination
and blocks analysis for this frozen preview protocol.

The user service `om6dof-dd-gng.service` launches that legacy YOLO process with
`Restart=on-failure`; it must be stopped for the complete frozen run and may be
restored afterward. Both the launch guard and preflight test this condition.
