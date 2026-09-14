"""Replay a captured scene on an isolated reachability node; no action clients."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from om6dof_dd_gng.msg import EnvironmentGraph,EnvironmentNode,ReachabilityGraph,ReachabilityPlan
from sensor_msgs.msg import JointState
p=Path('/tmp/oriented_pick_diagnosis');data=json.loads((p/'snapshot.json').read_text())
args=json.loads((p/'replay_args.json').read_text())
assert args[0].endswith('/reachability_graph_node')
args=[a.replace(':=/om6dof_topo_gng_v2/',':=/topology_motion_replay/') for a in args]
(p/'center_target_replay_args.json').write_text(json.dumps(args))
args+=['-r','/joint_states:=/topology_motion_replay/joints','-p','gng_debug_publish_training_samples:=false','-p','include_target_in_collision:=true']
for param,topic in {'environment_graph_topic':'environment_graph_data','graph_data_topic':'reachability_graph_data','plan_topic':'reachability_plan','marker_topic':'reachability_graph','path_topic':'reachability_path','query_topic':'reachability_query','training_samples_topic':'training_samples','training_samples_cloud_topic':'training_cloud','rebuild_service':'rebuild','plan_service':'plan','scene_validation_service':'validate'}.items():
 args += ['-p', f'{param}:=/topology_motion_replay/{topic}']
args += ['-p','graph_method:=workspace_samples','-p','workspace_samples_file:=/tmp/oriented_pick_diagnosis/green/workspace_samples.csv','-p','sample_count:=3198','-p','target_refinement_enabled:=false']
args += ['-p','exact_max_replans:=200','-p','target_node_selection:=component_center']
rclpy.init();n=rclpy.create_node('topology_motion_replay_driver')
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
received={}
for key,typ,topic in [('graph',ReachabilityGraph,'reachability_graph_data'),('plan',ReachabilityPlan,'reachability_plan')]:
 n.create_subscription(typ,'/topology_motion_replay/'+topic,lambda m,k=key:received.update({k:message_to_ordereddict(m)}),q)
ep=n.create_publisher(EnvironmentGraph,'/topology_motion_replay/environment_graph_data',2)
jp=n.create_publisher(JointState,'/topology_motion_replay/joints',2)
e=EnvironmentGraph();set_message_fields(e,data['environment'])
if os.environ.get('INJECT_START_COLLISION') == '1':
 obstacle=EnvironmentNode();obstacle.id=99999999;obstacle.class_id=-1
 set_message_fields(obstacle.position,data['plan']['end_effector_path']['poses'][0]['pose']['position'])
 e.nodes.append(obstacle)
j=JointState();j.name=data['plan']['joint_path_preview']['joint_names'];j.position=data['plan']['joint_path_preview']['points'][0]['positions']
log=(p/'center_target_replay.log').open('w');proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
try:
 start=time.monotonic();last=0;report=0
 while time.monotonic()-start<110:
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()-last>.8:
   j.header.stamp=n.get_clock().now().to_msg();e.header.stamp=j.header.stamp;jp.publish(j);ep.publish(e);last=time.monotonic()
  if time.monotonic()-report>5:
   print('RECEIVED',list(received),flush=True);report=time.monotonic()
  plan=received.get('plan',{})
  if (bool(plan.get('reason')) and not plan.get('reason', '').startswith('waiting_')) and 'graph' in received:
   break
 (p/'center_target_replay_result.json').write_text(json.dumps(received))
 plan=received.get('plan',{})
 print(json.dumps({k:plan.get(k) for k in ['valid','reason','target_distance','reachability_node_ids','exact_state_checks','exact_replans','planning_time_ms']}),flush=True)
 if plan.get('valid'):
  graph=received['graph'];edges={tuple(sorted((x['source_id'],x['target_id']))) for x in graph['edges']};path=plan['reachability_node_ids'];assert all(tuple(sorted(pair)) in edges for pair in zip(path,path[1:]));print('PATH_EDGES_VERIFIED',len(path)-1,flush=True)
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 log.close();n.destroy_node();rclpy.shutdown()
