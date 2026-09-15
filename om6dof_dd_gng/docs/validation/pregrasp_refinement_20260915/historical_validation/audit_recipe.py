from pathlib import Path
import hashlib,importlib.util,json,math,sys,yaml,numpy as np
folder=Path(sys.argv[1] if len(sys.argv)>1 else '/tmp/om6dof_pregrasp_bridge_replay_g50r3hax')
r=json.loads((folder/'result.json').read_text());plan=r.get('frozen_validated_plan') or [p for p in r['plans']if p['valid']][-1]
p=yaml.safe_load((folder/'params.yaml').read_text())['reachability_graph_node']['ros__parameters']
model=folder/'audited_model.urdf';model.write_text(p['robot_description'])
script=Path('/home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/paper/icra2027_dual_graph_manipulation/scripts/check_witness_frames.py')
spec=importlib.util.spec_from_file_location('independent_fk',script);fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
chain=fk.load_chain(model);points=plan['joint_path_preview']['points'];names=plan['joint_path_preview']['joint_names'];count=plan['pregrasp_waypoint_count'];target=np.array([plan['grasp_target_position'][k]for k in 'xyz']);offset=np.array(p['grasp_tcp_to_pinch']) if'grasp_tcp_to_pinch'in p else np.zeros(3)
axis=np.array(p['pregrasp_tool_approach_axis']);axis=axis/np.linalg.norm(axis)
audit={'fixture_scope':'Historical complete scene, synthetic measured-state publishers. No physical execution.','model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'target_center_m':target.tolist(),'graph_node_ids':plan['reachability_node_ids'],'pregrasp_waypoint_count':count,'total_waypoint_count':len(points),'bridge_waypoint_count':next(d['pregrasp_bridge_waypoints'] for d in r['diagnostics'] if d.get('pregrasp_bridge_used')),'independent_fk':{}}
for name,index in [('pregrasp',count-1),('endpoint',len(points)-1)]:
 pose=fk.fk(chain,dict(zip(names,points[index]['positions'])));pinch=(pose@np.r_[offset,1])[:3];world_axis=pose[:3,:3]@axis;v=target-pinch;distance=np.linalg.norm(v)
 audit['independent_fk'][name]={'tcp_position_m':pose[:3,3].tolist(),'pinch_position_m':pinch.tolist(),'world_approach_axis':world_axis.tolist(),'distance_to_target_m':float(distance),'absolute_vertical_axis':float(abs(world_axis[2]))}
 if name=='pregrasp':audit['independent_fk'][name]['alignment_dot']=float(world_axis@(v/distance));audit['independent_fk'][name]['alignment_angle_deg']=math.degrees(math.acos(min(1,max(-1,world_axis@(v/distance)))))
audit['reported_grasp_position_error_m']=plan['grasp_position_error'];audit['endpoint_residual_difference_m']=abs(audit['reported_grasp_position_error_m']-audit['independent_fk']['endpoint']['distance_to_target_m'])
audit['strictly_increasing_times']=all(b['time_from_start']['sec']+b['time_from_start']['nanosec']*1e-9>a['time_from_start']['sec']+a['time_from_start']['nanosec']*1e-9 for a,b in zip(points,points[1:]))
(folder/'plan_audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(json.dumps(audit,indent=2))
