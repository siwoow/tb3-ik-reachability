"""Isaac Lab collection demo. Simulation only; no ROS extensions.
Physical mode: prescribed base trajectory; arm/gripper/object use PhysX.
Assisted mode: prescribed joints/object, object contacts disabled from start.
--assisted is an explicitly labelled animation alternative, NOT physical grasp proof.
"""
import argparse,json,sys,math,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--assisted',action='store_true');p.add_argument('--output',default=None)
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(p);args=p.parse_args();args.enable_cameras=True
launcher=AppLauncher(args);app=launcher.app
import numpy as np,torch
from pxr import UsdGeom,UsdPhysics,UsdShade,Gf,PhysxSchema
import isaaclab.sim as su
from isaaclab.sim.converters import UrdfConverter,UrdfConverterCfg
from isaaclab.assets import Articulation,ArticulationCfg,RigidObject,RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
import omni.replicator.core as rep
ROOT=Path(__file__).resolve().parents[1];OUT=Path(args.output or Path(__file__).resolve().parent/('assisted' if args.assisted else 'physical'));OUT.mkdir(parents=True,exist_ok=True)
plans=json.loads((Path(__file__).resolve().parent/'plans.json').read_text());pick=plans['pick']['best'];place=plans['place']['best']
report={'mode':'assisted visualization' if args.assisted else 'physical contact trial','base_control':'prescribed root trajectory; wheel traction/navigation NOT validated','object_position_source':'simulation ground truth, no vision','physical_grasp_success':False,'bin_placement_success':False,'events':[]}
video=None

def main():
 global video
 sim=su.SimulationContext(su.SimulationCfg(dt=1/120,device=args.device,render_interval=5));stage=sim.stage
 sim.set_camera_view([1.65,-1.65,1.15],[.55,0,.15])
 c=UrdfConverter(UrdfConverterCfg(asset_path=str(ROOT/'assets/resolved.urdf'),usd_dir=str(OUT/'robot'),usd_file_name='robot.usd',fix_base=False,merge_fixed_joints=False,force_usd_conversion=True,self_collision=True,collision_from_visuals=False,collider_type='convex_hull',convert_mimic_joints_to_normal_joints=False,joint_drive=UrdfConverterCfg.JointDriveCfg(target_type='position',gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=200.,damping=12.))))
 robot=Articulation(ArticulationCfg(prim_path='/World/Robot',spawn=su.UsdFileCfg(usd_path=c.usd_path),actuators={'arm':ImplicitActuatorCfg(joint_names_expr=['joint[1-4]'],stiffness=200.,damping=12.),'gripper':ImplicitActuatorCfg(joint_names_expr=['gripper_.*_joint'],stiffness=200.,damping=2.,effort_limit_sim=1.),'wheels':ImplicitActuatorCfg(joint_names_expr=['wheel_.*_joint'],stiffness=20.,damping=2.)}))
 su.GroundPlaneCfg(color=(.13,.17,.21)).func('/World/Ground',su.GroundPlaneCfg(color=(.13,.17,.21)))
 light=su.DomeLightCfg(intensity=1800.,color=(.85,.91,1.));light.func('/World/Light',light)
 def box(name,pos,size,color):
  cfg=su.CuboidCfg(size=size,collision_props=su.CollisionPropertiesCfg(),visual_material=su.PreviewSurfaceCfg(diffuse_color=color));cfg.func('/World/'+name,cfg,translation=pos)
 # Low presentation pad, plus open-top bin with real five-sided collision geometry.
 box('PickPad',(.5,0,.0175),(.17,.13,.035),(.38,.44,.52))
 box('BinFloor',(1.05,0,.0025),(.23,.19,.005),(.08,.38,.48))
 for name,pos,size in [('BinLeft',(1.05,.10,.05),(.25,.012,.10)),('BinRight',(1.05,-.10,.05),(.25,.012,.10)),('BinBack',(1.17,0,.05),(.012,.20,.10)),('BinFront',(.93,0,.007),(.012,.20,.014))]:box(name,pos,size,(.08,.48,.58))
 # A single rigid body with overlapping ellipsoidal convex banana segments (synthetic banana).
 banana=UsdGeom.Xform.Define(stage,'/World/Banana');banana.AddTranslateOp().Set(Gf.Vec3d(.5,0,.051))
 UsdPhysics.RigidBodyAPI.Apply(banana.GetPrim());mass=UsdPhysics.MassAPI.Apply(banana.GetPrim());mass.CreateMassAttr(.080)
 mat=UsdShade.Material.Define(stage,'/World/BananaMaterial');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(1.);pm.CreateDynamicFrictionAttr(.8);pm.CreateRestitutionAttr(0.)
 for i,x in enumerate(np.linspace(-.046,.046,7)):
  s=UsdGeom.Sphere.Define(stage,f'/World/Banana/segment{i}');s.CreateRadiusAttr(.015*(1-.2*abs(x/.046)));s.AddTranslateOp().Set(Gf.Vec3d(float(x),float(.008*(x/.046)**2),0.))
  s.CreateDisplayColorAttr([Gf.Vec3f(.95,.73,.035)]);UsdPhysics.CollisionAPI.Apply(s.GetPrim()).CreateCollisionEnabledAttr(not args.assisted);UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(mat,materialPurpose='physics')
 obj=RigidObject(RigidObjectCfg(prim_path='/World/Banana',spawn=None))
 cam=UsdGeom.Camera.Define(stage,'/World/ThirdPerson');cam.CreateFocalLengthAttr(34.);cam.CreateHorizontalApertureAttr(36.);cam.CreateClippingRangeAttr(Gf.Vec2f(.02,20.))
 view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.4,-1.0,1.7),Gf.Vec3d(.55,0,.15),Gf.Vec3d(0,0,1));cam.AddTransformOp().Set(view.GetInverse())
 product=rep.create.render_product(str(cam.GetPath()),(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 sim.reset();robot.update(1/120);obj.update(1/120)
 ids=[robot.joint_names.index('joint'+str(i)) for i in range(1,5)];gids=[i for i,n in enumerate(robot.joint_names) if n.startswith('gripper_')]
 report['joint_names']=robot.joint_names;report['arm_joint_limits']=robot.data.joint_pos_limits[0,ids].cpu().tolist()
 q=np.array(pick['q_pregrasp_rad']);opening=.019;base=0.;held=False;released=False;phase='INITIALIZE';ticks=0;frames=0
 positions=robot.data.default_joint_pos.clone();positions[:,ids]=torch.tensor(q,device=sim.device,dtype=positions.dtype);positions[:,gids]=opening
 robot.write_joint_state_to_sim(positions,torch.zeros_like(positions));sim.forward();robot.update(1/120)
 root0=robot.data.root_pose_w.clone();root0[:,:3]=torch.tensor([[0.,0.,.001]],device=sim.device)
 link=robot.body_names.index('link5');tcp_offset=torch.tensor([.126,0.,0.],device=sim.device)
 from isaaclab.utils.math import quat_apply
 # FFmpeg keeps compression off the simulation thread; text marks actual trial mode.
 title='KINEMATIC DEMO - NOT A PHYSICAL GRASP' if args.assisted else 'PHYSX CONTACT GRASP TRIAL'
 vf="drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='"+title+"':x=20:y=20:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='Prescribed base motion | simulation ground truth':x=20:y=52:fontsize=16:fontcolor=white:box=1:boxcolor=black@0.6"
 video=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','24','-i','-','-an','-vf',vf,'-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'collection.mp4')],stdin=subprocess.PIPE)
 stage.GetRootLayer().Export(str(OUT/'scene.usda'))
 def step():
  nonlocal ticks,frames
  positions[:,ids]=torch.tensor(q,device=sim.device,dtype=positions.dtype);positions[:,gids]=opening
  # Prescribed base motion isolates the manipulator contact experiment from a new navigation controller.
  root=root0.clone();root[:,0]=base;robot.write_root_pose_to_sim(root);robot.write_root_velocity_to_sim(torch.zeros((1,6),device=sim.device))
  for i,n in enumerate(robot.joint_names):
   if n.startswith('wheel_'):positions[:,i]=base/.033
  robot.set_joint_position_target(positions);robot.write_data_to_sim()
  if args.assisted:
   robot.write_joint_state_to_sim(positions,torch.zeros_like(positions));sim.forward();robot.update(1/120)
  if args.assisted and not held:
   pose=obj.data.root_pose_w.clone();pose[0,3:]=torch.tensor([1.,0.,0.,0.],device=sim.device)
   if not released:pose[0,:3]=torch.tensor([.5,0.,.05],device=sim.device)
   else:pose[0,:3]=torch.tensor([1.05,0.,max(.020,float(pose[0,2])-.0006)],device=sim.device)
   obj.write_root_pose_to_sim(pose);obj.write_root_velocity_to_sim(torch.zeros((1,6),device=sim.device))
  if args.assisted and held:
   pose=obj.data.root_pose_w.clone();pose[0,3:]=torch.tensor([1.,0.,0.,0.],device=sim.device);w=robot.data.body_link_pose_w[0,link];pose[0,:3]=w[:3]+quat_apply(w[3:].unsqueeze(0),tcp_offset.unsqueeze(0))[0];obj.write_root_pose_to_sim(pose);obj.write_root_velocity_to_sim(torch.zeros((1,6),device=sim.device))
  sim.step(render=ticks%5==0);robot.update(1/120);obj.update(1/120);ticks+=1
  if args.assisted:
   robot.write_root_pose_to_sim(root);robot.write_joint_state_to_sim(positions,torch.zeros_like(positions));sim.forward();robot.update(1/120)
  if ticks%5==1:
   data=rgb.get_data()
   if data.size:
    video.stdin.write(np.ascontiguousarray(data[:,:,:3]).tobytes());frames+=1
 def event(name):
  nonlocal phase
  phase=name;record={'phase':name,'time_s':ticks/120,'banana_xyz':obj.data.root_pos_w[0].cpu().tolist(),'base_x':base,'arm_q_actual':robot.data.joint_pos[0,ids].cpu().tolist(),'arm_q_command':q.tolist(),'tcp_xyz':(robot.data.body_link_pose_w[0,link,:3]+quat_apply(robot.data.body_link_pose_w[0,link,3:].unsqueeze(0),tcp_offset.unsqueeze(0))[0]).cpu().tolist()};report['events'].append(record);print(json.dumps(record),flush=True)
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
 move('BASE_TO_PICK',6.,target_base=pick['base_x_m'])
 move('PREGRASP',1.,target_q=pick['q_pregrasp_rad'])
 corridor=np.array(pick['corridor_q_rad'])
 for row in corridor[::-1]:move('DESCEND',.14,target_q=row)
 move('CLOSE_GRIPPER',1.5,target_open=max(-.010,pick['closing_m']-.002))
 initial_z=float(obj.data.root_pos_w[0,2]);held=args.assisted
 for row in corridor:move('LIFT',.16,target_q=row)
 move('HOLD',1.)
 lifted=float(obj.data.root_pos_w[0,2])-initial_z
 report['measured_lift_m']=lifted;report['physical_grasp_success']=bool(lifted>.035 and not args.assisted)
 if lifted<.035:
  event('FAILED_NO_PHYSICAL_LIFT');move('FAILURE_HOLD',2.);report['failure']='Banana did not rise 35 mm during gripper lift; no attachment used.'
 else:
  move('CARRY_TO_BIN',7.,target_base=place['base_x_m'])
  for row in np.array(place['corridor_q_rad'])[::-1]:move('LOWER_INTO_BIN',.16,target_q=row)
  move('RELEASE',1.5,target_open=.019);held=False;released=True
  for row in np.array(place['corridor_q_rad']):move('RETRACT',.14,target_q=row)
  move('RETURN_ARM',2.,target_q=place['q_pregrasp_rad']);move('COMPLETE',2.)
  xyz=obj.data.root_pos_w[0].cpu().numpy();report['final_banana_xyz']=xyz.tolist();report['bin_placement_success']=bool(abs(xyz[0]-1.05)<.105 and abs(xyz[1])<.085 and .005<xyz[2]<.10)
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
