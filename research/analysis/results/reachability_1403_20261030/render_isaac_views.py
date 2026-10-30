"""Render SAVED 1403 points in the actual Isaac Sim viewport, with timeline stopped.
No IK, FCL, physics steps, robot interface or camera acquisition is executed.
"""
import json,asyncio,math,sys
from pathlib import Path
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1920,'height':1400})
import omni.usd,omni.timeline
from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
from pxr import Usd,UsdGeom,UsdLux,Gf
import numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];meta=json.loads((HERE/'visualization_data.json').read_text());a=np.load(HERE/'render_data.npz');W,H=1920,1400
raw=HERE/'raw_isaac_viewport';raw.mkdir(exist_ok=True)
fontfile='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font=ImageFont.truetype(fontfile,25);small=ImageFont.truetype(fontfile,21);titlefont=ImageFont.truetype(fontfile,35)
try:
 omni.timeline.get_timeline_interface().stop();omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
 robot=stage.DefinePrim('/World/Robot','Xform');robot.GetReferences().AddReference(str(ROOT/'assets/usd/robot.usd'))
 for name,matrix in meta['robot_links_link1'].items():
  prim=stage.GetPrimAtPath('/World/Robot/'+name)
  if prim.IsValid():
   xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(matrix).T.reshape(-1).tolist()))
 # Actual meshes stay opaque, so the viewer can see which points are occluded by robot geometry.
 dome=UsdLux.DomeLight.Define(stage,'/World/Light');dome.CreateIntensityAttr(1600.)
 distant=UsdLux.DistantLight.Define(stage,'/World/Key');distant.CreateIntensityAttr(1200.);distant.AddRotateXYZOp().Set(Gf.Vec3f(30.,-25.,-30.))
 points=UsdGeom.Points.Define(stage,'/World/Saved1403TCP');points.CreatePointsAttr([Gf.Vec3f(*v) for v in a['points']]);points.CreateWidthsAttr([.0022]*1403);points.CreateDisplayColorAttr([Gf.Vec3f(.02,.65,.23)])
 # Metre tick grid and axes are display guides, not generated reachable samples.
 def curve(path,xyz,color,width=.0007):
  c=UsdGeom.BasisCurves.Define(stage,path);c.CreateTypeAttr('linear');c.CreateCurveVertexCountsAttr([len(xyz)]);c.CreatePointsAttr([Gf.Vec3f(*p) for p in xyz]);c.CreateWidthsAttr([width]);c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);return c
 axes=UsdGeom.Xform.Define(stage,'/World/Axes');origin=np.array([0.,0.,0.]);ends={'X':np.array([.56,0,0]),'Y':np.array([0,.23,0]),'Z':np.array([0,0,.52])};colors={'X':(.24,.29,.36),'Y':(.37,.42,.47),'Z':(.10,.33,.55)}
 for name,end in ends.items():
  curve('/World/Axes/'+name,[origin,end],colors[name],.001)
  axis='XYZ'.index(name)
  for j,t in enumerate(np.arange(.1,float(end[axis])+.001,.1)):
   pos=np.zeros(3);pos[axis]=t;d=np.zeros(3);d[1 if axis!=1 else 0]=.003
   curve(f'/World/Axes/{name}_tick{j}',[pos-d,pos+d],colors[name],.0007)
 # Floor plane outline, at true link1 ground height -0.101 m.
 curve('/World/GroundGuide',[[-.13,-.19,-.101],[.59,-.19,-.101],[.59,.19,-.101],[-.13,.19,-.101],[-.13,-.19,-.101]],(.71,.74,.78),.0006)
 banana=UsdGeom.Xform.Define(stage,'/World/ActualBananaCandidates')
 for c in meta['banana_candidates']:
  idx=c['id'];start=c['observed_link1'];end=c['target_link1']
  # Hollow rings are drawn for surface observations; solid sphere is the stored evaluated target.
  ring=[]
  for t in np.linspace(0,2*math.pi,33):ring.append(np.asarray(start)+[.003*math.cos(t),.003*math.sin(t),0])
  curve('/World/ActualBananaCandidates/'+idx+'_surface',ring,(.85,.025,.03),.0007)
  dot=UsdGeom.Sphere.Define(stage,'/World/ActualBananaCandidates/'+idx+'_target');dot.CreateRadiusAttr(.0035);dot.AddTranslateOp().Set(Gf.Vec3d(*end));dot.CreateDisplayColorAttr([Gf.Vec3f(.85,.025,.03)])
  curve('/World/ActualBananaCandidates/'+idx+'_offset',[start,end],(.85,.025,.03),.0006)
 banana.MakeInvisible()
 cam=UsdGeom.Camera.Define(stage,'/World/AnalysisCamera');cam.CreateFocalLengthAttr(35.);cam.CreateHorizontalApertureAttr(36.);cam.CreateVerticalApertureAttr(36.*H/W);cam.CreateClippingRangeAttr(Gf.Vec2f(.005,20.));cam.AddTransformOp()
 vp=get_active_viewport()
 if vp is None:raise RuntimeError('Isaac viewport is unavailable')
 vp.camera_path=str(cam.GetPath());vp.resolution=(W,H);vp.updates_enabled=True
 for _ in range(30):app.update()
 def view(eye,target,ortho=False,up=(0,0,1),width=.83):
  matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse();cam.GetOrderedXformOps()[0].Set(matrix);cam.CreateProjectionAttr('orthographic' if ortho else 'perspective');cam.GetHorizontalApertureAttr().Set(width*10 if ortho else 36.);cam.GetVerticalApertureAttr().Set((width*10 if ortho else 36.)*H/W)
 def project(p):
  local=cam.GetOrderedXformOps()[0].Get().GetInverse().Transform(Gf.Vec3d(*p));x,y,z=local
  if cam.GetProjectionAttr().Get()=='orthographic':sx=x/(cam.GetHorizontalApertureAttr().Get()/20);sy=y/(cam.GetVerticalApertureAttr().Get()/20)
  else:sx=x/-z*cam.GetFocalLengthAttr().Get()/(cam.GetHorizontalApertureAttr().Get()/2);sy=y/-z*cam.GetFocalLengthAttr().Get()/(cam.GetVerticalApertureAttr().Get()/2)
  return ((sx+1)*W/2,(1-sy)*H/2)
 def annotate(path,title,subtitle,comparison=False):
  im=Image.open(raw/path).convert('RGB');d=ImageDraw.Draw(im);d.rectangle((0,0,W,146),fill='white');d.text((30,16),title,font=titlefont,fill='#182432');d.text((30,68),subtitle,font=font,fill='#33485b');d.text((30,108),'좌표계 link1  |  단위 m  |  원본 1,403점 전부 사용  |  물리 파지 성공 표시 아님',font=small,fill='#485562')
  legend=[('#119653','초록: 저장된 IK + FCL 전체 경로 통과 (1,403개)'),('#e4b400','노랑: IK 성공 / FCL 실패 — 해당 행 저장 안 됨'),('#df3333','빨강: 저장된 실측 바나나 IK 실패 (비교 뷰 9개)')]
  boxh=108 if comparison else 76;d.rectangle((18,H-boxh-18,1235,H-12),fill='white')
  for j,(color,text) in enumerate(legend if comparison else legend[:2]):
   yy=H-boxh-8+j*29;d.ellipse((32,yy+5,47,yy+20),fill=color);d.text((60,yy),text,font=small,fill='#273746')
  for name,end in ends.items():
   x,y=project(end)
   if 20<x<W-160 and 160<y<H-150:d.text((x+7,y-12),name+' [m]',font=font,fill='#233e55',stroke_width=2,stroke_fill='white')
  for axis,vec in enumerate([ends['X'],ends['Y'],ends['Z']]):
   # Only non-degenerate projected axes get numerical tick labels.
   o=np.array(project([0,0,0]));e=np.array(project(vec))
   if np.linalg.norm(e-o)<60:continue
   for t in np.arange(.1,vec[axis]+.001,.1):
    p=np.zeros(3);p[axis]=t;x,y=project(p)
    if 20<x<W-80 and 160<y<H-150:d.text((x+5,y+6),f'{t:.1f}',font=small,fill='#40505f',stroke_width=1,stroke_fill='white')
  if comparison:
   # Separate labels for the tightly grouped saved real candidates, without changing their positions.
   for i,c in enumerate(meta['banana_candidates']):
    x,y=project(c['target_link1']);tx=W-310;ty=235+i*50
    d.line([(x,y),(tx-12,ty+12)],fill='#b85b5b',width=1);d.text((tx,ty),c['id']+'  IK 실패',font=small,fill='#c42c31')
   d.text((30,160),'빨간 원: 관측 표면  →  채운 점: 기존 +30mm X 적용 IK 목표',font=small,fill='#b12a30',stroke_width=2,stroke_fill='white')
  im.save(HERE/path)
 def capture(path,title,subtitle,comparison=False):
  if omni.timeline.get_timeline_interface().is_playing():raise RuntimeError('Physics timeline unexpectedly running')
  for _ in range(12):app.update()
  handle=capture_viewport_to_file(vp,str(raw/path));task=asyncio.ensure_future(handle.wait_for_result(completion_frames=3))
  for _ in range(600):
   app.update()
   if task.done():break
  if not task.done():raise RuntimeError('Isaac viewport capture timed out')
  task.result()
  # The capture future may finish before its asynchronous PNG write is flushed.
  from omni.kit.renderer_capture import acquire_renderer_capture_interface
  acquire_renderer_capture_interface().wait_async_capture()
  annotate(path,title,subtitle,comparison)
 view((.96,-.86,.72),(.16,0,.11));capture('01_reachability_3d.png','3D — 기존 TCP 파지 가능점 1,403개','TurtleBot3 + OpenMANIPULATOR-X · 원본 USD · 저장된 실제 관절 자세')
 view((.22,0,1.5),(.22,0,.12),True,(0,1,0),.85);capture('02_top_xy.png','Top View — XY 수평 분포','동일한 1,403개 점을 수직 투영 · 겹친 점도 모두 원본 데이터에 유지')
 view((.22,-1.5,.15),(.22,0,.15),True,(0,0,1),.85);capture('03_side_xz.png','Side View — XZ 높이별 분포','동일한 1,403개 점 · 바닥 높이 link1 Z = -0.101 m')
 banana.MakeVisible();view((.25,-1.6,.13),(.25,0,.13),True,(0,0,1),.90);capture('04_banana_comparison.png','실측 바나나 후보 9개와 저장된 파지 영역 비교','2026-10-29 21:13:14 KST RGB-D · 촬영 시점의 팔 관절·보정·TF 유지',True)
 stage.GetRootLayer().Export(str(HERE/'reachability_1403.usda'))
 # A compact comparison sheet assembled only from the actual Isaac renders above.
 sheet=Image.new('RGB',(2400,1810),'white');d=ImageDraw.Draw(sheet);d.text((25,12),'저장된 TCP 파지점 1,403개와 실측 바나나 후보 | link1 [m]',font=titlefont,fill='#1c3447')
 for n,file in enumerate(['01_reachability_3d.png','02_top_xy.png','03_side_xz.png','04_banana_comparison.png']):
  im=Image.open(HERE/file).convert('RGB');im.thumbnail((1190,865),Image.Resampling.LANCZOS);sheet.paste(im,(10+(n%2)*1200,70+(n//2)*870))
 sheet.save(HERE/'00_comparison_sheet.png')
 (HERE/'render_provenance.json').write_text(json.dumps({'renderer':'Isaac Sim 5.1 actual viewport','timeline':'stopped','new_IK_calculations':0,'new_point_samples':0,'point_count':1403,'robot_usd':str(ROOT/'assets/usd/robot.usd'),'robot_pose':'existing capture joint state; original FK places USD links for static display only','raw_unannotated_captures':str(raw),'annotations':'Scientific titles, axis units, saved statuses added with Pillow; no point geometry replaced.'},indent=2))
 print('Saved Isaac viewport views:',HERE,flush=True)
except Exception:
 import traceback;traceback.print_exc();raise
finally:app.close(wait_for_replicator=False,skip_cleanup=True)
