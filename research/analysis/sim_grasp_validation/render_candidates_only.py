"""Render existing candidates in Isaac Sim only. Timeline stays STOPPED.
No IK calculation, physics stepping, gripper control, or hardware connection.
"""
import json,math,subprocess,asyncio
from pathlib import Path
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':720})
import omni.usd,omni.timeline
import omni.replicator.core as rep
from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
from pxr import UsdGeom,Gf,Usd
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'candidate_visualization';OUT.mkdir(exist_ok=True)
source=ROOT/'pad_candidate1/scene.usda';data=json.loads((ROOT/'candidates.json').read_text())
video=None
try:
 omni.timeline.get_timeline_interface().stop()
 omni.usd.get_context().open_stage(str(source))
 for _ in range(30):app.update()
 stage=omni.usd.get_context().get_stage();cam=UsdGeom.Camera(stage.GetPrimAtPath('/World/ThirdPerson'))
 vp=get_active_viewport()
 if vp is None:raise RuntimeError('No active Isaac viewport')
 vp.camera_path=str(cam.GetPath());vp.resolution=(1280,720);vp.updates_enabled=True
 product=rep.create.render_product(str(cam.GetPath()),(1280,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 workspace=UsdGeom.Imageable(stage.GetPrimAtPath('/World/Workspace'));workspace.MakeInvisible()
 markers=UsdGeom.Imageable(stage.GetPrimAtPath('/World/Candidates'));markers.MakeVisible()
 # Project DISPLAY MARKERS onto the upper surface of the existing banana mesh.
 # Original planner TCP targets and IK/FCL results are copied without alteration.
 mesh=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Banana/Surface'));M=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
 v=np.array([list(M.Transform(Gf.Vec3d(*map(float,p)))) for p in mesh.GetPointsAttr().Get()]);indices=list(mesh.GetFaceVertexIndicesAttr().Get());counts=list(mesh.GetFaceVertexCountsAttr().Get());triangles=[];k=0
 for count in counts:
  face=indices[k:k+count];k+=count
  for j in range(1,count-1):triangles.append(v[[face[0],face[j],face[j+1]]])
 records=[]
 for c in data['candidates']:
  x,y,z=c['world_tcp_m'];hits=[]
  for tri in triangles:
   A=np.column_stack((tri[1,:2]-tri[0,:2],tri[2,:2]-tri[0,:2]))
   if abs(np.linalg.det(A))<1e-12:continue
   u,w=np.linalg.solve(A,np.array([x,y])-tri[0,:2])
   if u>=-1e-6 and w>=-1e-6 and u+w<=1+1e-6:hits.append(float(tri[0,2]+u*(tri[1,2]-tri[0,2])+w*(tri[2,2]-tri[0,2])))
  if not hits:raise RuntimeError(f'Candidate {c["id"]} does not project inside existing banana surface')
  surface=max(hits);i=c['id'];color=Gf.Vec3f(1.,.72,.02) if c['ik_fcl'] else Gf.Vec3f(.85,.035,.025)
  for prefix,height in [('P',surface),('Shaft',surface+.039),('Arrow',surface+.010)]:
   prim=stage.GetPrimAtPath(f'/World/Candidates/{prefix}{i}');UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(x,y,height));UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([color])
  for side in ['L','R']:UsdGeom.Imageable(stage.GetPrimAtPath(f'/World/Candidates/Jaw{i}_{side}')).MakeInvisible()
  records.append({'existing_candidate':c,'display_surface_xyz_m':[x,y,surface],'marker_projection_only':True,'physical_test_performed':False})
 (OUT/'display_metadata.json').write_text(json.dumps({'source_scene':str(source),'physics_timeline':'stopped','existing_candidates':records,'legend':{'yellow':'existing IK/FCL passed; not claiming physical success','red':'existing IK/FCL failed'},'projection_note':'Surface marker is a visual projection of unchanged TCP target XY onto the existing USD mesh; it is not a new IK/grasp target.'},indent=2))
 def view(eye,target):
  look=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1));cam.GetOrderedXformOps()[0].Set(look.GetInverse())
 def viewport_png(path):
  if omni.timeline.get_timeline_interface().is_playing():raise RuntimeError('Timeline unexpectedly running')
  for _ in range(4):app.update()
  handle=capture_viewport_to_file(vp,str(path));task=asyncio.ensure_future(handle.wait_for_result(completion_frames=2))
  for _ in range(500):
   app.update()
   if task.done():break
  if not task.done():raise RuntimeError('Viewport capture did not finish')
  task.result()
 def capture(name,eye,target):
  view(eye,target);viewport_png(OUT/name)
 capture('01_environment.png',(1.25,-1.,1.15),(.65,0,.12))
 capture('02_banana_candidates.png',(.65,-.24,.29),(.5,0,.085))
 workspace.MakeVisible();capture('03_workspace_candidates.png',(.92,-.75,.7),(.36,0,.18));workspace.MakeInvisible()
 view((.65,-.24,.29),(.5,0,.085));stage.GetRootLayer().Export(str(OUT/'candidate_scene.usda'))
 frames=OUT/'frames';frames.mkdir(exist_ok=True)
 for i in range(192):
  if i<48:view((1.25,-1.,1.15),(.65,0,.12))
  else:
   angle=-1.25+.6*(i-48)/143;view((.5+.29*math.cos(angle),.29*math.sin(angle),.28),(.5,0,.083))
  viewport_png(frames/f'{i:04d}.png')
 subprocess.run(['ffmpeg','-y','-loglevel','error','-framerate','24','-i',str(frames/'%04d.png'),'-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'candidate_visualization.mp4')],check=True)
 print(str(OUT),flush=True)
except Exception:
 import traceback;traceback.print_exc()
finally:
 if video is not None:video.stdin.close();video.wait()
 app.close(wait_for_replicator=False,skip_cleanup=True)
