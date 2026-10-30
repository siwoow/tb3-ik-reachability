"""Contact-only Isaac Sim validation. No robot/banana pose attachment; base trajectory prescribed.
Screenshots come exclusively from Isaac RTX render products.
"""
import argparse,json,sys,math,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=int,default=0);p.add_argument('--fixed-base',action='store_true');p.add_argument('--pad-target',action='store_true');p.add_argument('--output',default=None)
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(p);args=p.parse_args();args.enable_cameras=True;args.assisted=False
launcher=AppLauncher(args);app=launcher.app
import numpy as np,torch
from pxr import UsdGeom,UsdPhysics,UsdShade,Gf,PhysxSchema,Sdf,PhysicsSchemaTools
from PIL import Image
import isaaclab.sim as su
from isaaclab.sim.converters import UrdfConverter,UrdfConverterCfg
from isaaclab.assets import Articulation,ArticulationCfg,RigidObject,RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import ContactSensor,ContactSensorCfg
import omni.replicator.core as rep
ROOT=Path(__file__).resolve().parents[1];OUT=Path(args.output or Path(__file__).resolve().parent/('candidate_'+str(args.candidate)));OUT.mkdir(parents=True,exist_ok=True)
plans=json.loads((ROOT/'sim_collection/plans.json').read_text());pick=plans['pick']['best'];place=plans['place']['best']
candidate_data=json.loads(Path(__file__).with_name('pad_candidates.json' if args.pad_target else 'candidates.json').read_text());selected=candidate_data['candidates'][args.candidate]
if not selected['ik_fcl']:raise RuntimeError('Selected candidate failed IK/FCL')
pick.update(selected['plan'])
report={'candidate':selected,'structural_filtered_pairs':candidate_data['structural_filtered_pairs'],'mode':'assisted visualization' if args.assisted else 'physical contact trial','base_control':'PhysX wheel velocity drives with simulation-pose feedback; no root pose overwrites during execution','object_position_source':'simulation ground truth, no vision','physical_grasp_success':False,'fixed_base_validation':args.fixed_base,'bin_placement_success':False,'events':[]}
video=None

def main():
 global video
 sim=su.SimulationContext(su.SimulationCfg(dt=1/120,device=args.device,render_interval=5));stage=sim.stage
 sim.set_camera_view([1.65,-1.65,1.15],[.55,0,.15])
 c=UrdfConverter(UrdfConverterCfg(asset_path=str(ROOT/'assets/resolved.urdf'),usd_dir=str(OUT/'robot'),usd_file_name='robot.usd',fix_base=args.fixed_base,merge_fixed_joints=True,force_usd_conversion=True,make_instanceable=False,self_collision=True,collision_from_visuals=False,collider_type='convex_hull',convert_mimic_joints_to_normal_joints=False,joint_drive=UrdfConverterCfg.JointDriveCfg(target_type='position',gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=200.,damping=12.))))
 robot=Articulation(ArticulationCfg(prim_path='/World/Robot',init_state=ArticulationCfg.InitialStateCfg(pos=(.35 if args.fixed_base else 0.,0.,0.)),spawn=su.UsdFileCfg(usd_path=c.usd_path,activate_contact_sensors=True,collision_props=su.CollisionPropertiesCfg(contact_offset=.001,rest_offset=0.)),actuators={'arm':ImplicitActuatorCfg(joint_names_expr=['joint[1-4]'],stiffness=200.,damping=12.),'gripper':ImplicitActuatorCfg(joint_names_expr=['gripper_.*_joint'],stiffness=1000.,damping=5.,effort_limit_sim=1.),'wheels':ImplicitActuatorCfg(joint_names_expr=['wheel_.*_joint'],stiffness=0.,damping=1.)}))
 # Deinstance only this composed experiment, then author precise contact offsets.
 for prim in list(stage.Traverse()):
  if prim.IsInstance():prim.SetInstanceable(False)
 offsets=[]
 for prim in stage.Traverse():
  if str(prim.GetPath()).startswith('/World/Robot') and prim.HasAPI(UsdPhysics.CollisionAPI):
   api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(.001);api.CreateRestOffsetAttr(0.);offsets.append(str(prim.GetPath()))
 report['contact_offset_1mm_colliders']=offsets
 cache=UsdGeom.XformCache();bounds={}
 for prim in stage.Traverse():
  if 'gripper_' in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
   pts=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get());bounds[str(prim.GetPath())]={'vertices_min':pts.min(0).tolist(),'vertices_max':pts.max(0).tolist()}
 report['finger_collision_mesh_local_bounds']=bounds
 filtered=[]
 for a,b in candidate_data['structural_filtered_pairs']:
  pa=stage.GetPrimAtPath('/World/Robot/'+a);pb=stage.GetPrimAtPath('/World/Robot/'+b)
  if pa.IsValid() and pb.IsValid():
   UsdPhysics.FilteredPairsAPI.Apply(pa).CreateFilteredPairsRel().AddTarget(pb.GetPath());filtered.append([a,b])
 report['applied_structural_pairs']=filtered
 floor=su.CuboidCfg(size=(4.,3.,.02),collision_props=su.CollisionPropertiesCfg(),visual_material=su.PreviewSurfaceCfg(diffuse_color=(.28,.30,.32),roughness=.8));floor.func('/World/Floor',floor,translation=(.5,0,-.01))
 light=su.DomeLightCfg(intensity=1800.,color=(.85,.91,1.));light.func('/World/Light',light)
 def box(name,pos,size,color):
  cfg=su.CuboidCfg(size=size,collision_props=su.CollisionPropertiesCfg(),visual_material=su.PreviewSurfaceCfg(diffuse_color=color));cfg.func('/World/'+name,cfg,translation=pos)
 # Low presentation pad, plus open-top bin with real five-sided collision geometry.
 box('PickPad',(.5,0,.0175),(.12,.10,.035),(.38,.44,.52))
 box('BinFloor',(1.05,0,.0025),(.14,.18,.005),(.08,.38,.48))
 for name,pos,size in [('BinLeft',(1.05,.10,.05),(.15,.012,.10)),('BinRight',(1.05,-.10,.05),(.15,.012,.10)),('BinBack',(1.125,0,.05),(.012,.18,.10)),('BinFront',(.975,0,.007),(.012,.18,.014))]:box(name,pos,size,(.08,.48,.58))
 # A single rigid body with overlapping ellipsoidal convex banana segments (synthetic banana).
 banana=UsdGeom.Xform.Define(stage,'/World/Banana');banana.AddTranslateOp().Set(Gf.Vec3d(.5,0,.051))
 UsdPhysics.RigidBodyAPI.Apply(banana.GetPrim());mass=UsdPhysics.MassAPI.Apply(banana.GetPrim());mass.CreateMassAttr(.080)
 mat=UsdShade.Material.Define(stage,'/World/BananaMaterial');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(1.);pm.CreateDynamicFrictionAttr(.8);pm.CreateRestitutionAttr(0.)
 for i,x in enumerate(np.linspace(-.046,.046,7)):
  s=UsdGeom.Sphere.Define(stage,f'/World/Banana/segment{i}');s.CreateRadiusAttr(.015*(1-.2*abs(x/.046)));s.AddTranslateOp().Set(Gf.Vec3d(float(x),float(.008*(x/.046)**2),0.))
  s.CreateDisplayColorAttr([Gf.Vec3f(.95,.73,.035)]);UsdPhysics.CollisionAPI.Apply(s.GetPrim());pc=PhysxSchema.PhysxCollisionAPI.Apply(s.GetPrim());pc.CreateContactOffsetAttr(.001);pc.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(mat,materialPurpose='physics')
 for child in banana.GetPrim().GetChildren():UsdGeom.Imageable(child).MakeInvisible()
 surface=UsdGeom.Mesh.Define(stage,'/World/Banana/Surface');vertices=[];faces=[]
 for ring in range(25):
  u=-1.+2.*ring/24.;radius=.015*math.sqrt(max(.005,1.-u*u))
  for j in range(20):
   theta=2*math.pi*j/20;vertices.append(Gf.Vec3f(.058*u,.011*u*u+radius*math.cos(theta),radius*math.sin(theta)))
 for ring in range(24):
  for j in range(20):faces.extend([ring*20+j,ring*20+(j+1)%20,(ring+1)*20+(j+1)%20,(ring+1)*20+j])
 surface.CreatePointsAttr(vertices);surface.CreateFaceVertexCountsAttr([4]*(24*20));surface.CreateFaceVertexIndicesAttr(faces);surface.CreateSubdivisionSchemeAttr('catmullClark');surface.CreateDisplayColorAttr([Gf.Vec3f(.92,.68,.025)])
 obj=RigidObject(RigidObjectCfg(prim_path='/World/Banana',spawn=None))
 finger_sensors=[ContactSensor(ContactSensorCfg(prim_path='/World/Robot/gripper_'+side+'_link',filter_prim_paths_expr=['/World/Banana'])) for side in ['left','right']]
 hold_samples=[]
 cam=UsdGeom.Camera.Define(stage,'/World/ThirdPerson');cam.CreateFocalLengthAttr(34.);cam.CreateHorizontalApertureAttr(36.);cam.CreateClippingRangeAttr(Gf.Vec2f(.02,20.))
 view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.4,-1.0,1.7),Gf.Vec3d(.55,0,.15),Gf.Vec3d(0,0,1));cam.AddTransformOp().Set(view.GetInverse())
 markers=UsdGeom.Xform.Define(stage,'/World/Candidates')
 for candidate in candidate_data['candidates']:
  i=candidate['id'];x,y,z=candidate.get('world_body_m',candidate['world_tcp_m']);color=(1.,.72,.02) if candidate['ik_fcl'] else (.85,.035,.025)
  dot=UsdGeom.Sphere.Define(stage,f'/World/Candidates/P{i}');dot.CreateRadiusAttr(.0035);dot.AddTranslateOp().Set(Gf.Vec3d(x,y,z+.016));dot.CreateDisplayColorAttr([Gf.Vec3f(*color)])
  shaft=UsdGeom.Cylinder.Define(stage,f'/World/Candidates/Shaft{i}');shaft.CreateRadiusAttr(.0012);shaft.CreateHeightAttr(.045);shaft.AddTranslateOp().Set(Gf.Vec3d(x,y,z+.055));shaft.CreateDisplayColorAttr([Gf.Vec3f(*color)])
  tip=UsdGeom.Cone.Define(stage,f'/World/Candidates/Arrow{i}');tip.CreateRadiusAttr(.004);tip.CreateHeightAttr(.012);tip.AddTranslateOp().Set(Gf.Vec3d(x,y,z+.026));tip.AddRotateXOp().Set(180.);tip.CreateDisplayColorAttr([Gf.Vec3f(*color)])
  # Two parallel lines show the planned finger closing axis (+/-Y at q1=0).
  for side in [-1,1]:
   finger=UsdGeom.Cube.Define(stage,f'/World/Candidates/Jaw{i}_{"L" if side==1 else "R"}');finger.CreateSizeAttr(1.);finger.AddTranslateOp().Set(Gf.Vec3d(x,y+side*.021,z+.083));finger.AddScaleOp().Set(Gf.Vec3f(.007,.001,.012));finger.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 workspace=UsdGeom.Points.Define(stage,'/World/Workspace')
 cloud=np.genfromtxt(ROOT/'results/reachable_workspace.csv',delimiter=',',names=True);cloud=cloud[::max(1,len(cloud)//3500)]
 report['workspace_columns']=list(cloud.dtype.names)
 xyz=np.column_stack([cloud[k] for k in ['x_m','y_m','z_m']]);xyz[:,0]+=(.35 if args.fixed_base else 0.)
 workspace.CreatePointsAttr([Gf.Vec3f(*v) for v in xyz]);workspace.CreateWidthsAttr([.0018]*len(xyz));workspace.CreateDisplayColorAttr([Gf.Vec3f(.12,.35,.75)]);workspace.MakeInvisible()
 product=rep.create.render_product(str(cam.GetPath()),(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 contact_totals={}
 for prim in stage.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
 import omni.physx
 def contacts(headers,data):
  for h in headers:
   if h.num_contact_data:
    pair=' | '.join(sorted([str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]))
    impulse=sum(float(np.linalg.norm(list(data[k].impulse))) for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
    contact_totals[pair]=max(contact_totals.get(pair,0.),impulse)
 contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contacts)
 sim.reset();robot.update(1/120);obj.update(1/120)
 ids=[robot.joint_names.index('joint'+str(i)) for i in range(1,5)];gids=[i for i,n in enumerate(robot.joint_names) if n.startswith('gripper_')]
 report['joint_names']=robot.joint_names;report['masses']=dict(zip(robot.body_names,robot.data.default_mass[0].cpu().tolist()));report['arm_joint_limits']=robot.data.joint_pos_limits[0,ids].cpu().tolist()
 q=np.array(pick['q_pregrasp_rad']);opening=.019;base=.35 if args.fixed_base else 0.;held=False;released=False;phase='INITIALIZE';ticks=0;frames=0
 positions=robot.data.default_joint_pos.clone();positions[:,ids]=torch.tensor(q,device=sim.device,dtype=positions.dtype);positions[:,gids]=opening
 robot.write_joint_state_to_sim(positions,torch.zeros_like(positions));sim.forward();robot.update(1/120)
 root0=robot.data.root_pose_w.clone();report['initial_root_pose']=root0.cpu().tolist();root0[:,0:2]=0.
 link=robot.body_names.index('link5');tcp_offset=torch.tensor([.126,0.,0.],device=sim.device)
 from isaaclab.utils.math import quat_apply
 # FFmpeg keeps compression off the simulation thread; text marks actual trial mode.
 video=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','24','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'collection.mp4')],stdin=subprocess.PIPE)
 stage.GetRootLayer().Export(str(OUT/'scene.usda'))
 def snapshot(name,eye=None,target=None):
  if 'workspace' in name:
   cloud_world=xyz.copy();cloud_world[:,0]+=float(robot.data.root_pos_w[0,0])-(.35 if args.fixed_base else 0.);cloud_world[:,1]+=float(robot.data.root_pos_w[0,1]);workspace.GetPointsAttr().Set([Gf.Vec3f(*v) for v in cloud_world])
  if eye is not None:
   view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1));cam.GetOrderedXformOps()[0].Set(view.GetInverse())
  for _ in range(8):sim.render()
  data=rgb.get_data()
  if data.size:Image.fromarray(np.asarray(data[:,:,:3])).save(OUT/(name+'.png'))
 def recolor(status):
  color=(.05,.8,.12) if status=='success' else (.85,.035,.025)
  for stem in ['P','Shaft','Arrow','Jaw']:
   for prim in markers.GetPrim().GetChildren():
    if prim.GetName() in [stem+str(args.candidate),stem+str(args.candidate)+'_L',stem+str(args.candidate)+'_R']:UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
 snapshot('01_environment')
 snapshot('02_grasp_candidates',(.63,-.23,.30),(.5,0,.065))
 workspace.MakeVisible();snapshot('03_workspace_ik',(.95,-.90,.85),(.35,0,.20));workspace.MakeInvisible()
 markers.MakeInvisible();snapshot('00_video_view',(1.4,-1.,1.7),(.55,0,.15))
 def step():
  nonlocal ticks,frames
  positions[:,ids]=torch.tensor(q,device=sim.device,dtype=positions.dtype);positions[:,gids]=opening
  actual=robot.data.root_pose_w[0];w,x,y,z=actual[3:].cpu().tolist();yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
  velocity=max(-.05,min(.05,1.5*(base-float(actual[0]))))
  omega=max(-.25,min(.25,-2.*yaw-2.*float(actual[1])))
  wheel_ids=[robot.joint_names.index('wheel_left_joint'),robot.joint_names.index('wheel_right_joint')]
  wheels=torch.tensor([[(velocity-.144*omega)/.033,(velocity+.144*omega)/.033]],device=sim.device,dtype=positions.dtype)
  robot.set_joint_velocity_target(wheels,joint_ids=wheel_ids)
  robot.set_joint_position_target(positions);robot.write_data_to_sim()
  sim.step(render=ticks%5==0);robot.update(1/120);obj.update(1/120);ticks+=1
  for sensor in finger_sensors:sensor.update(1/120)
  if phase=='HOLD':
   w=robot.data.body_link_pose_w[0,link];tcp=w[:3]+quat_apply(w[3:].unsqueeze(0),tcp_offset.unsqueeze(0))[0]
   hold_samples.append({'z_m':float(obj.data.root_pos_w[0,2]),'tcp_distance_m':float(torch.norm(obj.data.root_pos_w[0]-tcp)),'finger_contact_N':[float(torch.norm(sensor.data.force_matrix_w)) for sensor in finger_sensors]})
  if ticks%5==1:
   data=rgb.get_data()
   if data.size:
    video.stdin.write(np.ascontiguousarray(data[:,:,:3]).tobytes());frames+=1
 def event(name):
  nonlocal phase
  phase=name;record={'phase':name,'time_s':ticks/120,'banana_xyz':obj.data.root_pos_w[0].cpu().tolist(),'base_x':base,'root_pose':robot.data.root_pose_w.cpu().tolist(),'arm_q_actual':robot.data.joint_pos[0,ids].cpu().tolist(),'arm_q_command':q.tolist(),'peak_contact_impulses':dict(sorted(contact_totals.items(),key=lambda kv:-kv[1])[:12]),'arm_effort':robot.data.applied_torque[0,ids].cpu().tolist(),'gripper_q':robot.data.joint_pos[0,gids].cpu().tolist(),'finger_poses':{n:robot.data.body_link_pose_w[0,robot.body_names.index(n)].cpu().tolist() for n in ['gripper_left_link','gripper_right_link']},'finger_contacts_N':[float(torch.norm(sensor.data.force_matrix_w)) for sensor in finger_sensors],'tcp_xyz':(robot.data.body_link_pose_w[0,link,:3]+quat_apply(robot.data.body_link_pose_w[0,link,3:].unsqueeze(0),tcp_offset.unsqueeze(0))[0]).cpu().tolist()};report['events'].append(record);print(json.dumps(record),flush=True)
 def move(name,duration,target_q=None,target_base=None,target_open=None):
  nonlocal q,base,opening
  event(name);q0=q.copy();b0=base;g0=opening
  for t in np.linspace(0,1,int(duration*120)):
   a=t*t*(3-2*t)
   if target_q is not None:q=q0*(1-a)+np.array(target_q)*a
   if target_base is not None:base=b0*(1-a)+target_base*a
   if target_open is not None:opening=g0*(1-a)+target_open*a
   step()
 move('INITIAL_SCENE',1.)
 
 if not args.fixed_base:move('BASE_TO_PICK',10.,target_base=pick['base_x_m']);move('NAV_SETTLE',3.)
 move('PREGRASP',2.,target_q=pick['q_pregrasp_rad'])
 corridor=np.array(pick['corridor_q_rad'])
 for row in corridor[::-1]:move('DESCEND',.14,target_q=row)
 move('CLOSE_GRIPPER',1.5,target_open=-.0083)
 event('CLOSED_CONTACT');snapshot('04_closed_contact',(.64,-.25,.23),(.5,0,.06));snapshot('00_restore_video',(1.4,-1.,1.7),(.55,0,.15))
 initial_z=float(obj.data.root_pos_w[0,2]);held=False
 for row in corridor:move('LIFT',.16,target_q=row)
 move('HOLD',1.)
 lifted=float(obj.data.root_pos_w[0,2])-initial_z
 report['measured_lift_m']=lifted;report['hold_samples']=hold_samples;report['physical_grasp_success']=bool(lifted>.035 and hold_samples and min(v['z_m']-initial_z for v in hold_samples)>.035 and max(v['tcp_distance_m'] for v in hold_samples)<.04 and all(max(v['finger_contact_N'][side] for v in hold_samples)>.01 for side in [0,1]))
 if not report['physical_grasp_success']:
  recolor('failure');markers.MakeVisible();snapshot('04_failed_grasp',(.68,-.35,.42),(.49,0,.08));workspace.MakeVisible();snapshot('03_workspace_ik_result',(.95,-.90,.85),(.35,0,.20));workspace.MakeInvisible();markers.MakeInvisible();snapshot('00_failure_video_view',(1.4,-1.,1.7),(.55,0,.15))
  event('FAILED_NO_PHYSICAL_LIFT');move('FAILURE_HOLD',2.);report['failure']='Banana did not rise 35 mm during gripper lift; no attachment used.'
 else:
  snapshot('04_physical_lift',(.70,-.40,.40),(.5,0,.12));recolor('success');markers.MakeVisible();snapshot('02_verified_grasp_points',(.63,-.23,.30),(.5,0,.065));workspace.MakeVisible();snapshot('03_workspace_ik_result',(.95,-.90,.85),(.35,0,.20));workspace.MakeInvisible();markers.MakeInvisible();snapshot('00_carry_video_view',(1.4,-1.,1.7),(.55,0,.15))
  if not args.fixed_base:
   move('CARRY_TO_BIN',16.,target_base=place['base_x_m']);move('NAV_SETTLE',3.)
   for row in np.array(place['corridor_q_rad'])[::-1]:move('LOWER_INTO_BIN',.16,target_q=row)
   move('RELEASE',1.5,target_open=.019);held=False;released=True
   for row in np.array(place['corridor_q_rad']):move('RETRACT',.14,target_q=row)
   move('RETURN_ARM',2.,target_q=place['q_pregrasp_rad']);move('COMPLETE',2.)
   xyz=obj.data.root_pos_w[0].cpu().numpy();report['final_banana_xyz']=xyz.tolist();report['bin_placement_success']=bool(abs(xyz[0]-1.05)<.105 and abs(xyz[1])<.085 and .005<xyz[2]<.10)
 snapshot('05_final_state');stage.GetRootLayer().Export(str(OUT/'result_scene.usda'))
 report['frames']=frames;report['video_seconds']=frames/24
 video.stdin.close();video.wait();video=None
 print(json.dumps(report,indent=2),flush=True)
try:main()
except Exception as e:
 report['error']=repr(e);raise
finally:
 if video is not None:
  video.stdin.close();video.wait()
 (OUT/'result.json').write_text(json.dumps(report,indent=2))
 app.close(wait_for_replicator=False,skip_cleanup=True)
