import sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tb3_grasp.model import RobotModel
from tb3_grasp.planner import check_grasp,GraspPolicy
m=RobotModel();c=[]
for i,dx in enumerate([0.,-.015,.015,-.030,.030]):
    target=np.array([.5+dx,.008*(dx/.046)**2,.05]);local=target-np.array([.35,0,0]);p=check_grasp(m,local,GraspPolicy())
    c.append({'id':i,'world_tcp_m':target.tolist(),'base_footprint_tcp_m':local.tolist(),'link1_tcp_m':(local-np.array([-.092,0,.101])).tolist(),'pitch_rad':float(np.pi/2),'source':'previous simulation center' if i==0 else 'adjusted along synthetic banana body','ik_fcl':p is not None,'plan':p,'physical_status':'unverified'})
rigid={'base_footprint':'base_footprint'}
for j in m.order:rigid[j.child]=rigid[j.parent] if j.kind=='fixed' else j.child
pairs=set(tuple(sorted(x)) for x in m.exemptions)
for a in rigid:
 for b in rigid:
  if a!=b and rigid[a]==rigid[b]:pairs.add(tuple(sorted([a,b])))
Path(__file__).with_name('candidates.json').write_text(json.dumps({'candidates':c,'structural_filtered_pairs':sorted(pairs),'model_sha256':m.sha256,'tcp_link5_offset_m':[.126,0,0],'real_camera_observations_reused':False},indent=2))
print([(x['id'],x['ik_fcl']) for x in c])
