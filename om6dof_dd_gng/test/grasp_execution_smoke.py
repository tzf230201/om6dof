#!/usr/bin/env python3
"""Headless synthetic ROS smoke, domain 218; no robot-action clients or hardware.

The default exercises selected-component exclusion.  Use
``--strict-target-collision`` to retain the original target-protected smoke.
All obstacle fixtures are synthetic and all measured joint states are simulated.
"""
from pathlib import Path
import argparse,copy,csv,hashlib,importlib.util,json,os,signal,subprocess,tempfile,time
import yaml

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--strict-target-collision',action='store_true',
                    help='Keep selected target geometry in all open-gripper collision checks')
args=parser.parse_args()
exclude_selected_target=not args.strict_target_collision

os.environ['ROS_DOMAIN_ID']='218'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
repo=Path(__file__).resolve().parents[2]
run=Path(tempfile.mkdtemp(prefix='om6dof_grasp_execution_smoke_'))
os.environ['ROS_LOG_DIR']=str(run/'roslog')
source=repo/'experiments/cartesian_workspace/results/comparison_v1_v2_25mm_20260909/v2'
with (source/'points.csv').open() as f:
 reader=csv.DictReader(f);fields=reader.fieldnames
 rows=[r for r in reader if r['status']=='pose_found' and 174.999<float(r['x_mm'])<300.001 and abs(float(r['y_mm']))<50.001 and 99.999<float(r['z_mm'])<200.001]
with (run/'subset.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
initial=next(r for r in rows if all(abs(float(r[f'{c}_mm'])-v)<1e-6 for c,v in zip('xyz',[225,0,150])))
spec=importlib.util.spec_from_file_location('experiment_launch',repo/'om6dof_dd_gng/launch/ddgng_experiment_reachability.launch.py')
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
config=launch.prepare_config(repo/'om6dof_dd_gng/config/topo_gng_v2.yaml',run/'subset.csv',run,final_grasp=True)
doc=yaml.safe_load(config.read_text());p=doc['reachability_graph_node']['ros__parameters']
p.update(robot_description=(source/'model.urdf').read_text(),robot_description_semantic=(repo/'om6dof_moveit_config/config/om6dof.srdf').read_text(),planning_period_sec=.3)
p['exclude_selected_target_from_collision']=exclude_selected_target
for name in ['environment_graph_topic','joint_state_topic','graph_data_topic','training_samples_topic','training_samples_cloud_topic','marker_topic','plan_topic','path_topic','query_topic','rebuild_service','plan_service','scene_validation_service']:
 p[name]='/grasp_smoke/'+name
p['expanded_urdf_sha256']=hashlib.sha256(p['robot_description'].encode()).hexdigest()
p['srdf_sha256']=hashlib.sha256(p['robot_description_semantic'].encode()).hexdigest()
p['reachability_parameters_sha256']=hashlib.sha256(yaml.safe_dump(p,sort_keys=True).encode()).hexdigest()
config.write_text(yaml.safe_dump({'reachability_graph_node':{'ros__parameters':p}},sort_keys=False))

import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import JointState
from om6dof_dd_gng.msg import EnvironmentGraph,EnvironmentNode,TopologyEdge,ReachabilityPlan
from om6dof_dd_gng.srv import ValidateGraspExecution
from trajectory_msgs.msg import JointTrajectory

from ament_index_python.packages import get_package_prefix
binary=Path(get_package_prefix('om6dof_dd_gng'))/'lib/om6dof_dd_gng/reachability_graph_node'
result={'run_directory':str(run),'domain_id':218,'binary':str(binary.resolve()),'binary_mtime':binary.stat().st_mtime,'subset_count':len(rows),'hardware_access':False,'action_clients_created':False,'exclude_selected_target_from_collision':exclude_selected_target,'checks':[]}
node=None;proc=None;log=None
try:
 rclpy.init();node=rclpy.create_node('grasp_synthetic_smoke_client')
 pubj=node.create_publisher(JointState,p['joint_state_topic'],20)
 pube=node.create_publisher(EnvironmentGraph,p['environment_graph_topic'],10)
 plans=[]
 qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 sub=node.create_subscription(ReachabilityPlan,p['plan_topic'],lambda x:plans.append(x),qos)
 client=node.create_client(ValidateGraspExecution,'/grasp_smoke/validate_grasp_execution')
 current=[float(initial[f'q{i}_rad']) for i in range(1,7)]
 names=[f'joint{i}' for i in range(1,7)]
 environment_variant='base'
 def publish_joints():
  m=JointState();m.header.stamp=node.get_clock().now().to_msg();m.name=names+['gripper_left_joint','gripper_right_joint'];m.position=current+[.019,.019];pubj.publish(m)
 def publish_environment():
  m=EnvironmentGraph();m.header.frame_id='world';m.header.stamp=node.get_clock().now().to_msg()
  # A lower-ID, unreachable same-class fragment must not hijack selection.
  # The real three-node target consequently has a nonzero component index.
  fragment=EnvironmentNode();fragment.id=0;fragment.class_id=39;fragment.confidence=1.
  fragment.position.x=2.;fragment.position.y=2.;fragment.position.z=2.;m.nodes.append(fragment)
  for i,z in enumerate([.14,.15,.16],1):
   n=EnvironmentNode();n.id=i;n.class_id=39;n.confidence=1.;n.position.x=.295;n.position.z=z;m.nodes.append(n)
  for a,b in [(1,2),(2,3)]:
   e=TopologyEdge();e.source_id=a;e.target_id=b;m.edges.append(e)
  # Symmetric additions keep the selected component's AABB centre unchanged.
  # The exact same colliding point is subsequently relabelled unknown or made
  # a disconnected same-class component, providing a protected-scene control.
  if environment_variant.startswith('selected_finger'):
   additions=[(20,39,(.295,.040,.15)),(21,39,(.295,-.040,.15))]
  elif environment_variant.startswith('selected_wrist'):
   additions=[(20,39,(.225,0.,.15)),(21,39,(.365,0.,.15))]
  elif environment_variant=='unknown_finger':
   additions=[(99,-1,(.295,.040,.15))]
  elif environment_variant=='same_class_finger':
   additions=[(99,39,(.295,.040,.15)),(100,39,(.295,.050,.15))]
  elif environment_variant=='unknown_wrist':
   additions=[(99,-1,(.225,0.,.15))]
  else:
   additions=[]
  for identifier,class_id,position in additions:
   n=EnvironmentNode();n.id=identifier;n.class_id=class_id;n.confidence=1.
   n.position.x,n.position.y,n.position.z=position;m.nodes.append(n)
   if environment_variant.startswith('selected_'):
    e=TopologyEdge();e.source_id=2;e.target_id=identifier;m.edges.append(e)
  if environment_variant=='same_class_finger':
   e=TopologyEdge();e.source_id=99;e.target_id=100;m.edges.append(e)
  pube.publish(m)
 tj=node.create_timer(.05,publish_joints);te=node.create_timer(.1,publish_environment)
 log=(run/'planner.log').open('w')
 proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config),'-r','/om6dof_topo_gng_v2/validate_grasp_execution:=/grasp_smoke/validate_grasp_execution'],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
 result['own_subprocess_pid']=proc.pid
 def pump(seconds):
  until=time.monotonic()+seconds
  while time.monotonic()<until:
   if proc.poll() is not None:raise RuntimeError('Planner exited '+str(proc.returncode))
   rclpy.spin_once(node,timeout_sec=.02)
 deadline=time.monotonic()+70
 plan=None
 while time.monotonic()<deadline:
  pump(.1)
  candidates=[x for x in plans if x.valid and x.grasp_approach_valid]
  if candidates:plan=copy.deepcopy(candidates[-1]);break
 if plan is None:
  result['seen_plan_reasons']=sorted(set(x.reason for x in plans))
  raise RuntimeError('No valid full-grasp preview within70seconds')
 result['plan']={'reason':plan.reason,'valid':plan.valid,'grasp_approach_valid':plan.grasp_approach_valid,'pregrasp_waypoint_count':plan.pregrasp_waypoint_count,'trajectory_points':len(plan.joint_path_preview.points),'grasp_position_error_m':plan.grasp_position_error,'target':[plan.grasp_target_position.x,plan.grasp_target_position.y,plan.grasp_target_position.z],'target_environment_node_id':plan.target_environment_node_id,'selected_target_excluded_from_collision':plan.selected_target_excluded_from_collision}
 if plan.selected_target_excluded_from_collision!=exclude_selected_target:
  raise RuntimeError('Preview collision-policy contract does not match configured policy')
 if plan.target_environment_node_id not in {1,2,3}:
  raise RuntimeError('Lower-ID same-class fragment hijacked the intended target component')
 deadline=time.monotonic()+5
 while not client.service_is_ready() and time.monotonic()<deadline:pump(.1)
 if not client.service_is_ready():raise RuntimeError('Validation service unavailable')
 def check(label,points,closure,expect,reason_contains=''):
  req=ValidateGraspExecution.Request();req.trajectory=copy.deepcopy(plan.joint_path_preview);req.trajectory.points=copy.deepcopy(points)
  req.target_position=copy.deepcopy(plan.grasp_target_position);req.target_class_id=39;req.target_environment_node_id=plan.target_environment_node_id
  req.gripper_open_position=.019;req.gripper_close_position=-.010;req.include_closure=closure
  pump(.35);f=client.call_async(req);deadline=time.monotonic()+8
  while not f.done() and time.monotonic()<deadline:pump(.03)
  if not f.done():raise RuntimeError(label+': service timeout')
  response=f.result();entry={'stage':label,'environment_variant':environment_variant,'valid':response.valid,'reason':response.reason,'expected_valid':expect,'expected_reason_contains':reason_contains,'passed':response.valid==expect and reason_contains in response.reason};result['checks'].append(entry);print(json.dumps(entry),flush=True)
  if not entry['passed']:raise RuntimeError(label+': unexpected validation '+response.reason)
 count=plan.pregrasp_waypoint_count;points=plan.joint_path_preview.points
 check('graph',points[:count],False,True)
 current=list(points[count-1].positions)
 check('approach_and_closure_sweep',points[count-1:],True,True)
 current=list(points[-1].positions)
 check('closure_only',points[-1:],True,True)
 # include_closure=False deliberately uses strict open-gripper validation.
 # This proves component exclusion, rather than the separate close-phase
 # finger/target contact allowance, is responsible for target acceptance.
 environment_variant='selected_finger'
 check('selected_component_at_open_finger',points[-1:],False,exclude_selected_target,
       '' if exclude_selected_target else 'grasp_execution_collision:')
 environment_variant='selected_wrist'
 check('selected_component_at_wrist',points[-1:],False,exclude_selected_target,
       '' if exclude_selected_target else 'grasp_execution_collision:')
 environment_variant='unknown_finger'
 check('unknown_obstacle_at_open_finger',points[-1:],False,False,
       'grasp_execution_collision:dd_gng_grasp_obstacles')
 environment_variant='same_class_finger'
 check('disconnected_same_class_at_open_finger',points[-1:],False,False,
       'grasp_execution_collision:dd_gng_grasp_obstacles')
 environment_variant='unknown_wrist'
 check('unknown_obstacle_at_wrist',points[-1:],False,False,
       'grasp_execution_collision:dd_gng_grasp_obstacles')
 result['passed']=True
except Exception as exc:
 result['passed']=False;result['error']=str(exc)
finally:
 if proc is not None and proc.poll() is None:
  proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 if log:log.close()
 if node is not None:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 (run/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2),flush=True)
 print('Planner log:',run/'planner.log',flush=True)
raise SystemExit(0 if result.get('passed') else 1)
