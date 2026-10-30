"""Visualize saved results only: NO IK, collision evaluation or sampling."""
from pathlib import Path
import sys,json,csv,hashlib,shutil,xml.etree.ElementTree as ET
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.spatial import cKDTree
import trimesh
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];sys.path.insert(0,str(ROOT))
from tb3_grasp.model import RobotModel,origin
model=RobotModel();summary=json.loads((ROOT/'results/workspace_summary.json').read_text());ref=ROOT/'results/real_sim_comparison_20261029';stored=json.loads((ref/'comparison_data.json').read_text());capture=stored['capture'];a=np.load(ref/'current_capture/capture.npz');T=a['link1_base_footprint'];q=a['arm_joints'];opening=float(a['opening'])
assert model.sha256==summary['robot_model_sha256']==stored['model_urdf_sha256']
cloud=np.genfromtxt(ROOT/'results/grasp_workspace.csv',delimiter=',',names=True);assert len(cloud)==summary['feasible_grasp_samples']==1403
p_base=np.column_stack([cloud[n] for n in ['x_m','y_m','z_m']]);p=p_base@T[:3,:3].T+T[:3,3]
with (HERE/'grasp_workspace_link1.csv').open('w') as f:
 writer=csv.writer(f);writer.writerow(['source_row_1based','link1_x_m','link1_y_m','link1_z_m',*cloud.dtype.names,'saved_ik_fcl_full_path_pass'])
 for i,row in enumerate(cloud):writer.writerow([i+1,*p[i],*[row[k].item() for k in cloud.dtype.names],1])
nearest=cKDTree(p);candidates=[]
for i,(c,evaluations) in enumerate(zip(stored['candidates'],stored['evaluations'])):
 statuses=set(r['status'] for r in evaluations)
 status='valid' if 'valid' in statuses else 'fcl_failure' if statuses-{'ik'} else 'ik_failure'
 best=next((r for r in evaluations if r['status']=='valid'),next((r for r in evaluations if r['pitch']==-45),evaluations[0]))
 dist,idx=nearest.query(c['target_link1']);candidates.append({'id':'B'+str(i+1),**c,'saved_status':status,'reason':best['reason'],'evaluated_pitch_degrees':[r['pitch'] for r in evaluations],'nearest_saved_tcp_distance_m':float(dist),'nearest_saved_row_1based':int(idx+1),'beyond_saved_xyz_bounds':bool(np.any(np.asarray(c['target_link1'])<p.min(0)) or np.any(np.asarray(c['target_link1'])>p.max(0)))})
with (HERE/'banana_candidates_link1.csv').open('w') as f:
 writer=csv.writer(f);writer.writerow(['id','observed_x_m','observed_y_m','observed_z_m','target_x_m','target_y_m','target_z_m','saved_status','reason','nearest_saved_tcp_distance_m'])
 for c in candidates:writer.writerow([c['id'],*c['observed_link1'],*c['target_link1'],c['saved_status'],c['reason'],c['nearest_saved_tcp_distance_m']])
# Original visual meshes at the STORED real arm pose; FK is used solely to place the model.
fk=model.fk([q],opening);root=ET.parse(model.urdf).getroot();vertices=[];faces=[];mesh_offsets=[]
for link in root.findall('link'):
 for visual in link.findall('visual'):
  g=visual.find('geometry');mesh=g.find('mesh')
  if mesh is not None:
   shape=trimesh.load_mesh(mesh.get('filename'),process=False);v=shape.vertices*np.array(list(map(float,mesh.get('scale','1 1 1').split())));f=shape.faces
  elif g.find('box') is not None:
   shape=trimesh.creation.box(extents=list(map(float,g.find('box').get('size').split())));v,f=shape.vertices,shape.faces
  elif g.find('cylinder') is not None:
   e=g.find('cylinder');shape=trimesh.creation.cylinder(radius=float(e.get('radius')),height=float(e.get('length')),sections=24);v,f=shape.vertices,shape.faces
  elif g.find('sphere') is not None:
   shape=trimesh.creation.icosphere(subdivisions=2,radius=float(g.find('sphere').get('radius')));v,f=shape.vertices,shape.faces
  else:continue
  matrix=T@fk[link.get('name')][0]@origin(visual);v=v@matrix[:3,:3].T+matrix[:3,3];offset=sum(len(x) for x in vertices);vertices.append(v);faces.append(f+offset);mesh_offsets.append({'link':link.get('name'),'vertices':len(v),'triangles':len(f)})
v=np.concatenate(vertices);f=np.concatenate(faces);tcp=(T@fk['end_effector_link'][0])[:3,3]
robot_links={name:(T@matrix[0]).tolist() for name,matrix in fk.items()}
time=datetime.fromtimestamp(capture['depth_stamp_ns']/1e9,ZoneInfo('Asia/Seoul')).isoformat()
meta={'frame_id':'link1','units':'m','point_count':len(p),'point_meaning':'desired grasp-center TCP positions, NOT mobile-base positions','classification':'all 1403 saved rows passed sampled IK/FCL pregrasp-descent-close-lift checks','physical_grasp_success_claimed':False,'source_frame_id':summary['frame_id'],'link1_from_base_footprint':T.tolist(),'cloud_bounds_link1_m':{'min':p.min(0).tolist(),'max':p.max(0).tolist()},'policy':summary['policy'],'robot_model_sha256':model.sha256,'robot_pose_source':'saved real camera capture; no new joint sampling','joint_positions_rad':q.tolist(),'opening_m':opening,'capture_time_KST':time,'capture_metadata':capture,'banana_candidates':candidates,'saved_failure_points_available_in_workspace_csv':False,'yellow_cloud_points':0,'red_cloud_points':0,'banana_target_offsets':'saved +0.030m link1 X from visible surface; shown separately from hollow surface markers','condition_difference':'Workspace: vertical tool pitch +90deg under model convention. Real candidates: saved native C++ pitches -80..-15deg; no new IK evaluations.','limitations':['Point samples do not certify a continuous volume or exact boundary.','Green is geometric full-path feasibility, NOT physical grasp success.','IK-only successes rejected by full FCL were not saved in grasp_workspace.csv, so no yellow samples are invented.','32,768 reachable_workspace rows are FK joint samples, not 1,403 IK grasp points; not mixed here.','Real capture camera calibration remains experimental; preserved capture TF/joints used without correction.','Visual meshes are unchanged original model; legacy camera geometry is not a fresh calibrated camera simulation.'],'robot_links_link1':robot_links,'current_model_tcp_link1':tcp.tolist(),'visual_meshes':mesh_offsets}
(HERE/'visualization_data.json').write_text(json.dumps(meta,indent=2))
np.savez_compressed(HERE/'render_data.npz',points=p,robot_vertices=v,robot_faces=f,arm_q=q,root=T,model_tcp=tcp)
original=HERE/'original_results';original.mkdir(exist_ok=True)
files=[ROOT/'results/grasp_workspace.csv',ROOT/'results/workspace_summary.json',ref/'comparison_data.json',ref/'current_capture/capture.json',ref/'current_capture/capture.npz']
manifest=[]
for file in files:
 dest=original/file.name;shutil.copy2(file,dest);manifest.append({'source':str(file),'copy':str(dest),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
(HERE/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
# Self-contained local Plotly bundle; all 1403 points and original model mesh retained.
shutil.copy2('/home/mobility/miniconda3/envs/moge/lib/python3.10/site-packages/plotly/package_data/plotly.min.js',HERE/'plotly.min.js')
def trace(points,**kw):return dict(x=points[:,0].tolist(),y=points[:,1].tolist(),z=points[:,2].tolist(),**kw)
traces=[trace(v,type='mesh3d',i=f[:,0].tolist(),j=f[:,1].tolist(),k=f[:,2].tolist(),name='Original URDF model / saved arm pose',color='#aeb7c1',opacity=.8,flatshading=True,hoverinfo='skip',showlegend=True),trace(p,type='scatter3d',mode='markers',name='1403 TCP: IK + FCL full path passed (geometric)',marker={'color':'#119653','size':3,'opacity':.85},customdata=list(range(1,1404)),hovertemplate='source row %{customdata}<br>link1 (%{x:.4f}, %{y:.4f}, %{z:.4f}) m<extra>saved IK/FCL pass</extra>')]
for status,color,name in [('valid','#119653','Real banana: saved IK/FCL pass'),('fcl_failure','#e4b400','Real banana: IK pass / FCL fail'),('ik_failure','#df3333','Real banana: saved IK fail')]:
 cs=[c for c in candidates if c['saved_status']==status]
 if cs:
  pts=np.array([c['target_link1'] for c in cs]);traces.append(trace(pts,type='scatter3d',mode='markers+text',name=name,marker={'color':color,'size':5,'symbol':'diamond'},text=[c['id'] for c in cs],textposition='top center',customdata=[c['reason'] for c in cs],hovertemplate='%{text}<br>(%{x:.4f}, %{y:.4f}, %{z:.4f}) m<br>%{customdata}<extra>saved C++ IK evaluation</extra>'))
observed=np.array([c['observed_link1'] for c in candidates]);traces.append(trace(observed,type='scatter3d',mode='markers',name='Real observed surface (before saved +30 mm X)',marker={'color':'#df3333','size':4,'symbol':'circle-open'}))
for c in candidates:traces.append(trace(np.array([c['observed_link1'],c['target_link1']]),type='scatter3d',mode='lines',line={'color':'#df3333','width':2},showlegend=False,hoverinfo='skip'))
traces.append(trace(tcp[None],type='scatter3d',mode='markers',name='Model TCP at saved arm pose',marker={'color':'#1662a3','size':5,'symbol':'cross'}))
layout={'title':{'text':'Saved 1,403 TCP grasp positions + real banana candidates | link1 [m]'},'scene':{'xaxis':{'title':'link1 X [m]'},'yaxis':{'title':'link1 Y [m]'},'zaxis':{'title':'link1 Z [m]'},'aspectmode':'data','camera':{'eye':{'x':1.5,'y':-2.0,'z':1.2}}},'margin':{'t':70,'l':0,'r':0,'b':0},'legend':{'orientation':'h','y':-.07},'paper_bgcolor':'white'}
html='''<!doctype html><html><head><meta charset="utf-8"><title>Saved reachability: 1403 TCP samples</title><script src="plotly.min.js"></script><style>body{margin:0;font:15px sans-serif}#plot{width:100vw;height:88vh}p{margin:8px 16px}button{padding:6px 12px;margin:4px}</style></head><body><p><b>기존 TCP 파지점 1,403개 — link1, m</b> · 초록: 저장된 IK/FCL 전 경로 통과(물리 성공 아님) · 빨강: 실제 바나나 후보의 저장된 IK 실패. 원형은 관측 표면, 마름모는 기존 +30mm X 적용 IK 목표. 드래그: 회전, 휠: 확대, 범례 클릭: 표시 전환.</p><button onclick="camera('3d')">3D</button><button onclick="camera('top')">Top XY</button><button onclick="camera('side')">Side XZ</button><div id="plot"></div><script>const traces=TRACES;const layout=LAYOUT;Plotly.newPlot('plot',traces,layout,{responsive:true,displaylogo:false});function camera(v){const x=v==='top'?{eye:{x:0,y:0,z:2.2},up:{x:0,y:1,z:0},projection:{type:'orthographic'}}:v==='side'?{eye:{x:0,y:-2.2,z:0},up:{x:0,y:0,z:1},projection:{type:'orthographic'}}:{eye:{x:1.5,y:-2,z:1.2},up:{x:0,y:0,z:1},projection:{type:'perspective'}};Plotly.relayout('plot',{'scene.camera':x});}</script></body></html>'''.replace('TRACES',json.dumps(traces,separators=(',',':'))).replace('LAYOUT',json.dumps(layout,separators=(',',':')))
(HERE/'interactive_reachability.html').write_text(html)
notes=f'''원본 파일 및 의미\n================\n기존 grasp_workspace.csv: {len(p)}개, 원래 base_footprint 프레임의 TCP 파지 중심. 베이스 후보 위치가 아님.\n기존 analyze_workspace.py는 check_grasp() 전체 경로 통과 행만 이 CSV에 저장. 따라서 전부 초록색: IK + FCL 접근/하강/닫기/인양 경로 통과. 물리 파지 성공을 뜻하지 않음.\n정책: 수직 +90도 tool pitch, 접근/인양 각 60mm, 30mm 구형 물체 폭 근사, 바닥 여유 3mm, 추가 자기 충돌 여유 0mm.\n실패한 격자점 좌표나 IK 통과/FCL 실패 행은 저장되지 않았으므로 추가 생성하지 않음.\n별도 reachable_workspace.csv 32768행은 FK 관절 샘플. 이 시각화의 1403개와 다른 자료이므로 섞지 않음.\n\n좌표 정렬\nlink1 = base_footprint + (0.092, 0, -0.101)m. 저장된 실제 TF 행렬 그대로 사용.\n3D, Top XY, Side XZ 모두 같은 1403개 점, 같은 link1 프레임, 단위 m.\n로봇 모델은 원본 URDF/USD와 저장된 2026-10-29 관절 자세로 표시. 새 FK 점 샘플링이나 IK 계산 없음.\n\n실제 바나나\n관측 시각: {time}\n동일한 한 장의 RGB-D에서 나온 후보 9개만 사용. 당시 팔 관절과 camera->link5->link1 변환은 원본 capture.json/npz에 보존.\n속이 빈 빨간 원: 관측 표면 observed_link1. 빨간 마름모: 기존 X +30mm 적용 target_link1, 저장된 실제 C++ IK 평가 목표.\n9개 모두 저장된 평가에서 IK 실패. FCL을 통과했다고 표시하지 않음.\n최근접 점 거리: {min(c['nearest_saved_tcp_distance_m'] for c in candidates):.4f}~{max(c['nearest_saved_tcp_distance_m'] for c in candidates):.4f}m. 단, 최근접 거리나 점군 외곽은 연속 도달 범위 판정 기준이 아님.\n점군은 수직 접근, 바나나 기록은 native -80~-15도 접근 평가여서 자세 조건 차이가 있음. 실제 실패 표시는 저장된 native 결과를 사용.\n\n색상 변경 주의\n이전 5개 시뮬레이션 후보 그림: 노랑 = IK/FCL 통과·물리 미검증, 빨강 = IK/FCL 실패.\n이번 연구 분석 범례: 초록 = 저장된 IK+FCL 전체 경로 통과(기하학), 노랑 = IK 성공/FCL 실패, 빨강 = IK 실패.\n원본 1403개에 노랑/빨강 행은 없으며 임의로 만들지 않음. 이번 빨강은 기존 실측 바나나 9개만 의미.\n\n새 로봇/카메라 실행, 물리 시험, 제어 코드 변경, 새 IK/충돌 계산, 무작위 샘플링 없음.\n'''
(HERE/'README_ko.txt').write_text(notes)
print(json.dumps({'saved_points':len(p),'actual_candidates':len(candidates),'bounds_link1':meta['cloud_bounds_link1_m'],'nearest_distance_m':[min(c['nearest_saved_tcp_distance_m'] for c in candidates),max(c['nearest_saved_tcp_distance_m'] for c in candidates)],'robot_visual_triangles':len(f),'output':str(HERE)},indent=2))
