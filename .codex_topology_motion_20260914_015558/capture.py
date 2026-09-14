import json,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from om6dof_dd_gng.msg import EnvironmentGraph,ReachabilityGraph,ReachabilityPlan
rclpy.init();n=rclpy.create_node('oriented_pick_readonly_capture');data={}
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
for key,typ,topic in [('environment',EnvironmentGraph,'environment_graph_data'),('graph',ReachabilityGraph,'reachability_graph_data'),('plan',ReachabilityPlan,'reachability_plan')]:
 n.create_subscription(typ,'/om6dof_topo_gng_v2/'+topic,lambda m,k=key:data.update({k:message_to_ordereddict(m)}),q if key!='environment' else QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
t=time.monotonic()
while time.monotonic()-t<8:
 rclpy.spin_once(n,timeout_sec=.1)
 if len(data)==3:break
Path('/tmp/oriented_pick_diagnosis/snapshot.json').write_text(json.dumps(data))
print({k:len(v.get('nodes',[])) for k,v in data.items()});print(data.get('plan',{}).get('reason'))
n.destroy_node();rclpy.shutdown()
