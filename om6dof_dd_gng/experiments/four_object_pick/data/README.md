# Recorded experiment data

`record_trial.sh` writes each run into a timestamped directory here. Raw ROS bags,
videos, telemetry, and process logs are intentionally ignored by Git because they
can be large. Keep the generated `metadata/`, CSV, JSONL, and summary files with
the raw run when archiving an experiment.

Directories beginning with `smoke_` validate instrumentation only. They are
not four-object trials and must not enter paper statistics. A
`run_complete.flag` means the recorder closed its required files and ROS bag;
it does not mean that objects were present or that perception/planning/picking
succeeded. Experimental readiness is reported separately in
`run_summary.json`. A normal run also contains `scene_readiness.json`; a smoke
run must show the semantic decision as explicitly skipped, not successful.

Recorder qualification is stored in
`smoke_instrumentation_clean_20260910_run_001/`. It passed recording integrity,
exclusive process ownership, and final topic-total cross-checks after the
competing web-stream service was stopped. It intentionally contains no staged
objects, detections, labels, or valid target plan and is therefore not a paper
trial. Earlier `smoke_live_*` captures are retained only as instrumentation
audit evidence and are superseded by the clean capture.
