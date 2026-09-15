std_msgs/Header header
string graph_method
uint64 graph_revision
uint64 query_id
string scene_id
uint32 requested_target_environment_node_id
geometry_msgs/Point requested_target_position
bool valid
string reason
uint32 blocked_node_count
uint32 blocked_edge_count
float64 planning_time_ms
bool exact_collision_valid
uint32 exact_state_checks
uint32 exact_replans
float64 exact_validation_time_ms
uint32 start_node_id
uint32 goal_node_id
uint32 target_environment_node_id
float64 target_distance
float64 graph_cost
float64 start_connection_cost
float64 total_joint_path_cost
uint32[] reachability_node_ids
trajectory_msgs/JointTrajectory joint_path_preview
nav_msgs/Path end_effector_path

# Opt-in pickup contract. The full preview includes graph traversal followed
# by a separately validated Cartesian insertion. Old graph-only plans leave
# these fields at defaults and must never authorize closing at the pregrasp.
bool grasp_approach_valid
# Prefix through the actual aligned pregrasp, including a validated local
# correction after the archived graph anchor when necessary.
uint32 pregrasp_waypoint_count
geometry_msgs/Point grasp_target_position
float64 grasp_position_error
float64 gripper_open_position
float64 gripper_close_position
# True only when the chosen connected target component was omitted from both
# capsule and mesh environment collision checks. Other components stay checked.
bool selected_target_excluded_from_collision
