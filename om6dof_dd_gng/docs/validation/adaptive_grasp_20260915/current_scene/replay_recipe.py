#!/usr/bin/env python3
from pathlib import Path
import argparse,copy,hashlib,json,os,signal,subprocess,tempfile,time
import yaml
parser=argparse.ArgumentParser();parser.add_argument('--capture-dir',default=str(Path(__file__).resolve().parent/'replay_fixture'));parser.add_argument('--base-params',default=str(Path(__file__).resolve().parent/'params.yaml'));parser.add_argument('--binary',default='/home/kublab/ros2_ws/install/om6dof_dd_gng/lib/om6dof_dd_gng/reachability_graph_node');parser.add_argument('--seconds',type=float,default=90);parser.add_argument('--pregrasp-refinement-enabled',choices=['true','false'],required=True);args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']='219';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
run=Path(tempfile.mkdtemp(prefix='om6dof_pregrasp_bridge_replay_'));os.environ['ROS_LOG_DIR']=str(run/'roslog')
capture=Path(args.capture_dir)
p=yaml.safe_load(Path(args.base_params).read_text())['reachability_graph_node']['ros__parameters']
import shutil
local_samples=Path(args.base_params).resolve().parent/'workspace_samples.csv'
if local_samples.exists():p['workspace_samples_file']=str(local_samples)
saved_samples=run/'workspace_samples.csv';shutil.copyfile(p['workspace_samples_file'],saved_samples);p['workspace_samples_file']=str(saved_samples)
for name in ['environment_graph_topic','joint_state_topic','graph_data_topic','training_samples_topic','training_samples_cloud_topic','marker_topic','plan_topic','path_topic','query_topic','rebuild_service','plan_service','scene_validation_service']:
 p[name]='/collision_replay/'+name
p['pregrasp_refinement_enabled']=args.pregrasp_refinement_enabled=='true'
p['reachability_parameters_sha256']=hashlib.sha256(yaml.safe_dump(p,sort_keys=True).encode()).hexdigest()
config=run/'params.yaml';config.write_text(yaml.safe_dump({'reachability_graph_node':{'ros__parameters':p}},sort_keys=False))
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from om6dof_dd_gng.msg import EnvironmentGraph,ReachabilityPlan
from om6dof_dd_gng.srv import ValidateGraspExecution
summary={'run_directory':str(run),'binary':str(Path(args.binary).resolve()),'domain':219,'obstacle_clearance':p['obstacle_clearance'],'actions_or_hardware':False,'plans':[],'diagnostics':[],'pregrasp_refinement_enabled':p['pregrasp_refinement_enabled'],'snapshot':'Current captured pregrasp_bridge_joint_jump scene, with measured arm values matching the retained failed-plan start. No physical execution.','execution_validation_checks':[]}
proc=None;node=None;log=None
try:
 rclpy.init();node=rclpy.create_node('collision_frozen_snapshot_replay')
 q=JointState();set_message_fields(q,json.loads((capture/'joints.json').read_text()))
 env=EnvironmentGraph();set_message_fields(env,json.loads((capture/'environment.json').read_text()))
 pubj=node.create_publisher(JointState,p['joint_state_topic'],10);pube=node.create_publisher(EnvironmentGraph,p['environment_graph_topic'],2)
 def pj():q.header.stamp=node.get_clock().now().to_msg();pubj.publish(q)
 def pe():env.header.stamp=node.get_clock().now().to_msg();pube.publish(env)
 node.create_timer(.05,pj);node.create_timer(.10,pe)
 def receive(m):
  if m.reason in ['waiting_for_robot_description','waiting_for_joint_states','waiting_for_environment_graph']:return
  d=message_to_ordereddict(m);summary['plans'].append(d);print(json.dumps({k:d[k] for k in ['valid','reason','blocked_node_count','blocked_edge_count','planning_time_ms','exact_replans','exact_state_checks']}),flush=True)
 node.create_subscription(ReachabilityPlan,p['plan_topic'],receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 node.create_subscription(String,p['plan_topic']+'/collision_diagnostics',lambda m:summary['diagnostics'].append(json.loads(m.data)),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 log=(run/'planner.log').open('w');proc=subprocess.Popen([args.binary,'--ros-args','--params-file',str(config),'-r','/om6dof_topo_gng_v2/validate_grasp_execution:=/collision_replay/validate_grasp_execution'],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());summary['own_subprocess_pid']=proc.pid
 deadline=time.monotonic()+args.seconds
 while time.monotonic()<deadline and (len(summary['plans'])<2 or not summary['diagnostics'] or summary['diagnostics'][-1].get('reason')!=summary['plans'][-1]['reason']):
  if proc.poll() is not None:raise RuntimeError('diagnostic planner exited '+str(proc.returncode))
  rclpy.spin_once(node,timeout_sec=.02)
 summary['complete']=bool(summary['plans'] and summary['diagnostics'] and summary['diagnostics'][-1].get('reason')==summary['plans'][-1]['reason'] and summary['plans'][-1]['reason']!='grasp_gripper_state_missing_or_stale')
 if summary['diagnostics']:print('DIAGNOSTICS',json.dumps(summary['diagnostics'][-1]),flush=True)
 if summary['plans'] and summary['plans'][-1]['valid']:
  frozen=ReachabilityPlan();set_message_fields(frozen,copy.deepcopy(summary['plans'][-1]))
  validation=node.create_client(ValidateGraspExecution,'/collision_replay/validate_grasp_execution')
  def pump(seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:
    if proc.poll() is not None:raise RuntimeError('Planner exited during validation')
    rclpy.spin_once(node,timeout_sec=.02)
  until=time.monotonic()+5
  while not validation.service_is_ready() and time.monotonic()<until:pump(.05)
  if not validation.service_is_ready():raise RuntimeError('Execution validation service unavailable')
  pts=frozen.joint_path_preview.points;c=frozen.pregrasp_waypoint_count
  for stage,segment,closure in [('graph_and_pregrasp_bridge',pts[:c],False),('insertion_and_closure_sweep',pts[c-1:],True),('closure_only',pts[-1:],True)]:
   for name,value in zip(frozen.joint_path_preview.joint_names,segment[0].positions):q.position[q.name.index(name)]=value
   for name in ['gripper_left_joint','gripper_right_joint']:
    if name in q.name:q.position[q.name.index(name)]=frozen.gripper_open_position
   pump(.35)
   req=ValidateGraspExecution.Request();req.trajectory=copy.deepcopy(frozen.joint_path_preview);req.trajectory.points=copy.deepcopy(segment)
   req.target_position=copy.deepcopy(frozen.grasp_target_position);req.target_class_id=39;req.target_environment_node_id=frozen.target_environment_node_id
   req.gripper_open_position=frozen.gripper_open_position;req.gripper_close_position=frozen.gripper_close_position;req.include_closure=closure
   pending=validation.call_async(req);until=time.monotonic()+8
   while not pending.done() and time.monotonic()<until:pump(.03)
   if not pending.done():raise RuntimeError(stage+': service timeout')
   response=pending.result();entry={'stage':stage,'valid':response.valid,'reason':response.reason,'synthetic_joint_state_only':True};summary['execution_validation_checks'].append(entry);print('VALIDATION',json.dumps(entry),flush=True)
   if not response.valid:raise RuntimeError(stage+': '+response.reason)
  summary['frozen_validated_plan']=message_to_ordereddict(frozen)

 if not summary['plans']:summary['error']='No completed plan within bound'
except Exception as exc:summary['error']=str(exc);summary['complete']=False
finally:
 if proc is not None and proc.poll() is None:
  proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 if log:log.close()
 if node is not None:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 (run/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
 print('Result:',run/'result.json',flush=True);print('Log:',run/'planner.log',flush=True)
 if (run/'planner.log').exists():
  for line in (run/'planner.log').read_text().splitlines():
   if 'DIAG_' in line:print(line,flush=True)
raise SystemExit(0 if summary.get('complete') else 1)
