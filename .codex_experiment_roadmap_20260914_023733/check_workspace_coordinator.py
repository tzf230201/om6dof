"""Offline integration fixture: geometry replay + synthetic track metadata, no ROS node."""
import json,time,threading,yaml
from types import SimpleNamespace
from rosidl_runtime_py.set_message import set_message_fields
from om6dof_dd_gng.msg import EnvironmentGraph,ReachabilityPlan
from sensor_msgs.msg import JointState
from om6dof_pick_and_place.graph_pick_node import GraphPickNode,ObjectTrackObservation,semantic_component
p='/tmp/oriented_pick_diagnosis/'
data=json.load(open(p+'snapshot.json'));replay=json.load(open(p+'workspace_success_result.json'))
e=EnvironmentGraph();set_message_fields(e,data['environment'])
r=ReachabilityPlan();set_message_fields(r,replay['plan'])
class_id,component,centroid=semantic_component(e,r.target_environment_node_id)
track=ObjectTrackObservation(1,class_id,'bottle',centroid,centroid,tuple(n.id for n in component))
j=JointState();j.name=list(r.joint_path_preview.joint_names);j.position=list(r.joint_path_preview.points[0].positions)
node=object.__new__(GraphPickNode);node._lock=threading.RLock();node.task_mode='move_to_target';node.world_frame='world';node.joint_names=list(j.name);node.tool_axis=[0,0,1];node.execution_enabled=False
params=yaml.safe_load(open('/home/kublab/ros2_ws/src/om6dof/om6dof_pick_and_place/config/graph_pick.yaml'))['graph_pick']['ros__parameters'];node.get_parameter=lambda key:SimpleNamespace(value=params[key]);node._bus_failure_counts=lambda:(0,0)
now=time.monotonic();node._input_snapshot=lambda:([(now,e)],r,now,{'accepted':True,'calibration_verified':True,'semantic_target_classes':'bottle'},now,{},now,[(now,[track])],j,now)
plan=node._build_plan();assert plan.task_mode=='move_to_target';assert plan.blockers==['execution_disabled_at_launch']
graph={n['id']:n for n in replay['graph']['nodes']}
assert list(plan.trajectory.points[0].positions)==list(j.position)
for point,node_id in zip(plan.trajectory.points[1:],r.reachability_node_ids):assert list(point.positions)==graph[node_id]['joint_positions']
print('COORDINATOR_REPLAY_ACCEPTED',{'points':len(plan.trajectory.points),'alignment_diagnostic':plan.alignment,'blockers':plan.blockers,'task_mode':plan.task_mode})
