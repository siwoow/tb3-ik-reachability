"""Verify the portable runtime/artifacts without importing Isaac or plot tools."""
import argparse
import hashlib
import importlib.abc
import json
from pathlib import Path
import platform
import sys


class NoSimulatorImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'isaaclab','isaacsim','torch','scipy','matplotlib','trimesh'}:
            raise ImportError('Runtime audit forbids simulator/analysis dependency: '+fullname)


def audit(planner_root):
    root=Path(planner_root).expanduser().resolve()
    sys.path.insert(0,str(root)); sys.meta_path.insert(0,NoSimulatorImports())
    import numpy as np
    from tb3_grasp.model import RobotModel
    from tb3_grasp.planner import check_grasp,plan_base_positions
    m=RobotModel(); cache=json.loads((root/'assets/collision_bounds.json').read_text())
    report={'python':sys.version,'architecture':platform.machine(),'numpy':np.__version__,
            'planner_root':str(root),'model_sha256':m.sha256,'collision_backend':m.collision_backend,
            'isaac_required':False,'arm_limits_rad':np.c_[m.lower,m.upper].tolist(),
            'model_root_frame':'base_footprint','base_link_height_offset_m':float(m.fk([[0,0,0,0]])['base_link'][0,2,3]),
            'shoulder_base_footprint_m':m.shoulder.tolist(),
            'cache_hash_matches_urdf':cache['resolved_urdf_sha256']==m.sha256,
            'runtime_artifacts':['tb3_grasp/','assets/resolved.urdf','assets/collision_bounds.json'],
            'optional_analysis_artifacts':['results/grasp_workspace.csv','results/workspace_summary.json'],
            'meshes_needed_for_runtime':False,'workspace_csv_used_as_live_ik_authority':False,
            'live_ik_equivalence_verified':False,'hardware_commands_sent':False}
    path=root/'results/grasp_workspace.csv'
    if path.exists():
        data=np.atleast_1d(np.genfromtxt(path,delimiter=',',names=True))
        fields=list(data.dtype.names)
        xyz=np.c_[data['x_m'],data['y_m'],data['z_m']]
        q=np.column_stack([data[f'q{i}_rad'] for i in range(1,5)])
        errors=np.linalg.norm(m.fk(q)['end_effector_link'][:,:3,3]-xyz,axis=1)
        report['workspace_csv']={'columns':fields,'rows':len(data),'finite':bool(np.isfinite(np.c_[xyz,q]).all()),
                                 'joint_limits_valid':bool(np.all(q>=m.lower-1.e-8) and np.all(q<=m.upper+1.e-8)),
                                 'max_saved_fk_error_m':float(errors.max()),'frame_id':'base_footprint','units':'m,rad'}
    solution=check_grasp(m,[.12,0,.05])
    report['known_grasp_reproduced']=solution is not None
    result=plan_base_positions([.5,0,.05],[0,0,0],model=m,radii=[.2],azimuth_samples=4,yaw_samples=4,top_k=1)
    report['known_base_plan_reproduced']=result['best'] is not None
    report['loaded_forbidden_modules']=[n for n in ('isaaclab','isaacsim','torch','scipy','matplotlib','trimesh') if n in sys.modules]
    report['runtime_ready']=bool(report['cache_hash_matches_urdf'] and report['known_grasp_reproduced']
                                 and report['known_base_plan_reproduced'] and not report['loaded_forbidden_modules']
                                 and m.fcl_shapes)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--planner-root',default='~/tb3_grasp_analysis');p.add_argument('--output')
    a=p.parse_args();report=audit(a.planner_root);text=json.dumps(report,indent=2)
    print(text)
    if a.output: Path(a.output).write_text(text)
    raise SystemExit(0 if report['runtime_ready'] else 2)
