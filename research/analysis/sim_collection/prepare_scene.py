"""Reuse the existing FCL planner; no ROS or hardware access."""
import sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from tb3_grasp.model import RobotModel
from tb3_grasp.planner import plan_base_positions,GraspPolicy
out=Path(__file__).resolve().parent
m=RobotModel(); plans={}
for name,xyz,start in [('pick',[.5,0,.05],[0,0,0]),('place',[1.05,0,.05],[.35,0,0])]:
    r=plan_base_positions(xyz,start,model=m,policy=GraspPolicy(),frame='simulation_world',radii=[.23],azimuth_samples=4,yaw_samples=1,top_k=1)
    if r['best'] is None: raise RuntimeError(name+' has no FCL-valid grasp corridor')
    plans[name]=r
(out/'plans.json').write_text(json.dumps(plans,indent=2))
print(json.dumps({n:p['best'] for n,p in plans.items()},indent=2))
