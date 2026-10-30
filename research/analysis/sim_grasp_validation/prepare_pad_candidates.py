import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tb3_grasp.model import RobotModel
from tb3_grasp.planner import check_grasp,GraspPolicy
m=RobotModel();data=json.loads(Path(__file__).with_name('candidates.json').read_text());shapes=json.loads((ROOT/'assets/collision_bounds.json').read_text())['shapes']
left=next(s for s in shapes if s['link']=='gripper_left_link');v=np.array(left['convex']['vertices']);distal=v[v[:,0]>.03];pad_x=.0817+float((distal[:,0].min()+distal[:,0].max())/2);offset=pad_x-.126
for c in data['candidates']:
 body=np.array(c['world_tcp_m']);c['world_body_m']=body.tolist();c['previous_world_tcp_m']=body.tolist();tcp=body+[0,0,offset];local=tcp-[.35,0,0];plan=check_grasp(m,local,GraspPolicy())
 if plan:
  qs=np.array(plan['corridor_q_rad']);actual=m.fk(qs)['end_effector_link'][:,:3,3]-[0,0,offset]
  a,_=m.screen(qs,.019,0.,0.,(body-[.35,0,0],.015,False),floor_margin=.003)
  b,_=m.screen(qs,-.0083,0.,0.,(actual,.015,True),floor_margin=.003)
  plan['actual_body_envelope_approach_clear']=bool(a.all());plan['actual_body_envelope_lift_clear']=bool(b.all())
  if not a.all() or not b.all():plan=None
 c.update(world_tcp_m=tcp.tolist(),base_footprint_tcp_m=local.tolist(),link1_tcp_m=(local-[-.092,0,.101]).tolist(),plan=plan,ik_fcl=plan is not None,physical_status='unverified',pad_center_link5_x_m=pad_x,tcp_to_pad_m=offset)
data['pad_offset_evidence']={'mesh_distal_x_minmax_m':[float(distal[:,0].min()),float(distal[:,0].max())],'gripper_joint_origin_link5_x_m':.0817,'tcp_link5_x_m':.126,'distal_pad_midpoint_link5_x_m':pad_x,'TCP_command_shift_world_z_m':offset,'interpretation':'Geometric midpoint of distal finger convex hull, contact success must still be measured.'}
Path(__file__).with_name('pad_candidates.json').write_text(json.dumps(data,indent=2));print(offset,[(c['id'],c['ik_fcl']) for c in data['candidates']])
