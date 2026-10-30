# TurtleBot3 + OpenMANIPULATOR-X 오프라인 파지 분석

기존 `project_bringup`, `project.launch.py`, 로봇 프로젝트와 원본 URDF를 수정하지 않는 독립 분석 도구입니다.
ROS 메시지를 발행하거나 실제 로봇을 움직이는 코드가 없습니다.

## 실행

현재 PC에는 `~/tb3_grasp_analysis/.venv`에 독립 분석 환경이 준비되어 있습니다.

```bash
cd ~/tb3_grasp_analysis
# 원본 모델 감사 + Mesh 경로만 해석한 별도 URDF 복사본 생성
.venv/bin/python prepare.py --urdf ~/robot_models/turtlebot3_manipulation.urdf

# Isaac Lab 실제 로딩, 관절 제한과 PhysX FK 검증
~/miniconda3/envs/env_isaaclab/bin/python isaac_validate.py --headless --device cpu

# 관절 공간 32,768점, 원통 좌표 목표 격자, 상부 접근/하강/닫기/인양 검사
.venv/bin/python analyze_workspace.py

# 실제 물체 좌표가 아닌 테스트 입력: odom 프레임, 단위 m 및 rad
.venv/bin/python plan_base.py --object .50 0 .05 --base 0 0 0 \
  --object-width .030 --approach-height .060 --output results/example_base_plan.json
.venv/bin/python visualize_plan.py results/example_base_plan.json

# Isaac GUI에서 원본 모델과 파지 가능 점 표시. 창을 닫으면 종료됩니다.
~/miniconda3/envs/env_isaaclab/bin/python isaac_validate.py --device cpu --show-workspace --hold

.venv/bin/python -m unittest discover -s tests -v
```

Isaac 스크립트는 별도 프로세스에서 실행합니다. 설치된 Isaac Sim 5.1 환경의 일반 종료 대기를 피하려고
headless 실행에서는 제공되는 `SimulationApp.close(skip_cleanup=True)`를 사용합니다.
검증 중에는 다양한 관절 자세를 직접 설정하므로 **실제 컨트롤러 추종 실험이 아닙니다.**

다른 PC에서 재현하려면 Python 가상환경에 `requirements.txt`를 설치하고, 아래 공식 저장소를
`vendor/turtlebot3_manipulation`에 복제합니다. 이번 분석의 고정 커밋은
`814f140e8e208994dd6159f189993156e3943d98`입니다. 원본 Mesh를 쓰는 대신 일관된 공식 Mesh 스냅샷을
별도 폴더에서 사용하며, 원본 파일과의 해시 차이는 `asset_report.json`에 명시합니다.

```bash
git clone --branch humble https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git vendor/turtlebot3_manipulation
git -C vendor/turtlebot3_manipulation checkout 814f140e8e208994dd6159f189993156e3943d98
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python prepare.py
```

## 파일

| 파일 | 역할 |
|---|---|
| `prepare.py` | 링크·관절·제한·mimic·Mesh 검사, 독립 URDF와 충돌 형상 캐시 생성 |
| `isaac_validate.py` | Isaac Lab 가져오기, USD 생성, PhysX 관절 제한/FK 확인, GUI 점 표시 |
| `tb3_grasp/model.py` | URDF 기반 FK, 실제 4-DOF 해석 IK, 보수적 충돌 검사 |
| `tb3_grasp/planner.py` | 상부 접근/하강/닫기/인양 검사, 베이스 후보와 순위 계산 |
| `analyze_workspace.py` | Workspace 계산, CSV·JSON·PNG·PDF 생성 |
| `plan_base.py` | 물체 좌표와 현재 베이스 자세를 입력받는 CLI |
| `visualize_plan.py` | 저장한 베이스 후보와 팔 접근 경로 그림 |
| `results/asset_report.json` | 원본 누락/해시 차이, 11개 Mesh, 링크·관절·충돌 형상 감사 |
| `results/isaac_validation.json` | 실제 Isaac 로딩과 PhysX 검증 결과 |
| `results/reachable_workspace.csv` | 위치 도달 샘플, 관절각, 충돌 검사 통과 여부 |
| `results/grasp_workspace.csv` | 접근부터 인양까지 통과한 목표점과 관절각·점수 |
| `results/workspace_summary.json` | 조건, 샘플 범위, 개수, 한계 |
| `results/workspace.png`, `grasp_top_view.png` | 3D 도달 영역/파지 영역, 평면도 |
| `results/example_base_plan.json`, `.csv`, `.png` | 테스트 물체 입력에 대한 베이스 후보와 상세 팔 자세 |

## 좌표 및 IK

원본 루트는 `base_footprint`이며, `base_link`는 그보다 10 mm 위입니다.
어깨 회전축 기준 위치는 `base_footprint`에서 `(-0.080, 0, 0.1775)` m입니다.
`base_link` 기준 좌표와 혼용하면 높이 오차가 발생합니다.

팔의 회전 관절은 yaw 1개와 평행한 pitch 3개입니다. 그리퍼는 별도 prismatic 관절이며
오른쪽 관절은 왼쪽 관절을 1:1로 모방합니다. 6-DOF 로봇처럼 XYZ와 임의의 roll/pitch/yaw를
동시에 독립 지정하지 않습니다. 로컬 tool +X를 아래로 향하게 하는 pitch=+π/2를 지정하고,
yaw는 어깨에서 목표점을 보는 방향에 결합합니다. 두 radial 방향과 두 elbow 해를 계산하고
원본 관절 제한 및 FK 잔차를 확인합니다. 특이점인 회전축 바로 위 목표는 보수적으로 거부합니다.

원본 제한은 joint1 `[-162°, 162°]`, joint2 `[-102.6°, 90°]`, joint3 `[-54°, 79.2°]`,
joint4 `[-102.6°, 117°]`입니다. 파지 후보에는 추가로 관절 한계에서 2° 여유를 요구합니다.
그리퍼 각 관절 제한은 `[-0.010, 0.019]` m입니다. 전체 손가락 경계로 계산한 보수적인 최대
내측 간격은 약 55.7 mm입니다. 닫기 목표는 이 경계에서 유도한 기하학적 값이며,
실제 패드 접촉 위치와 힘 제어는 보정/검증되지 않았습니다.

## 충돌과 실제 파지의 범위

기본 검사는 tool +X 수직 하향, 60 mm 위 사전 접근, 수직 하강, 그리퍼 닫기, 60 mm 인양입니다.
Cartesian 점 사이에도 최대 2° 간격으로 관절 보간을 검사합니다. 보간 TCP가 수직 경로에서
1 mm 이상 이탈하면 거부합니다. Cartesian 기본 간격은 5 mm이며, 접근과 인양 높이는 변경할 수 있습니다.
동일한 해석 IK branch를 유지해 elbow 전환을 허용하지 않습니다.

Mesh는 원본 충돌 Mesh 전체를 감싸는 볼록 껍질로, box/cylinder/sphere는 원래 기본 형상으로
FCL에서 검사합니다. 빠른 OBB 검사를 먼저 수행합니다. 이 방법은 빈 공간/오목한 부분 때문에
실제로 가능한 자세를 거부할 수 있습니다. FCL이 없는 환경에서는 더 보수적인 OBB 검사로
대체하며, 이 모델에서는 후보가 모두 사라질 수 있으므로 설치된 backend를 확인하세요.
물체는 기본 폭 30 mm의 구형 경계로 근사합니다. 손가락과의 접촉은 닫기/인양 단계에서만 허용합니다.
바나나의 전체 길이·곡률·무게·마찰·접촉력·토크는 검증하지 않습니다.

원본 SRDF에서 구조적 `Adjacent` 쌍만 제외합니다. `Never`/`User` 쌍은 추가로 검사합니다.
따라서 공식 MoveIt 설정보다 보수적입니다. 특히 이 URDF의 `base_scan`과 `link2`는
정면에서 약 0.9 mm밖에 떨어져 있지 않아 사용 가능한 yaw 영역이 크게 줄어듭니다.
Mesh 볼록 근사와 실제 라이다 장착 위치를 확인하기 전에는 충돌 제외를 추가하지 마세요.

기본값은 **원본 형상에 추가하는 자기 충돌 padding 0 mm**, **이동 링크 바닥 여유 3 mm**입니다.
원본 형상을 줄이거나 관절 제한을 넓히지 않습니다. 0 mm padding은 보정 오차/인식 불확실성의
여유가 있다는 뜻이 아닙니다. 추가 자기 충돌 여유를 명시하면 그대로 적용하고, 해가 없더라도 완화하지 않습니다.

```bash
# 추가 3 mm 자기 충돌 여유: 이 URDF에서는 라이다 간격 때문에 후보가 없어질 수 있습니다.
.venv/bin/python plan_base.py --object .50 0 .05 --base 0 0 0 \
  --collision-margin .003 --output results/strict_base_plan.json
```

파지 가능한 점은 샘플로 확인한 기하학적 후보입니다. 연속 공간 전체의 경계나 보장된 성공 확률이 아닙니다.
점수는 관절 제한 여유, 주요 자세에서의 자기 충돌 간격, 바닥 여유의 가중합입니다.
베이스 비용은 이동 거리 + 최종 회전량 + 차동구동 전후 회전량 + 낮은 파지 점수의 패널티입니다.
목표가 현재 베이스에서 이미 파지 가능하면 현재 위치도 후보에 포함합니다. 실패 시 후보 없음으로 반환합니다.

## Jetson / ROS2 연동

Jetson에서는 Isaac Lab 없이 다음 순수 계획 함수를 호출할 수 있습니다.
`tb3_grasp/`, `assets/resolved.urdf`, `assets/collision_bounds.json`을 같은 폴더 구조로 옮기고
NumPy 및 python-fcl을 설치하세요. Python-fcl의 ARM 패키지 제공 여부에 따라 소스 빌드가 필요할 수 있습니다.
FK/계획 단계는 Mesh를 직접 읽지 않고 해시가 일치하는 충돌 캐시를 사용합니다. 구조적 SRDF 제외 쌍도 캐시에 보존되어 있습니다.
Isaac 변환 또는 다른 원본 모델을 사용하려면 해당 위치에서 `prepare.py`를 다시 실행해야 합니다.

```python
from tb3_grasp.model import RobotModel
from tb3_grasp.planner import GraspPolicy, plan_base_positions

model = RobotModel()
result = plan_base_positions(
    object_world_xyz=[0.50, 0.0, 0.05],   # 원하는 TCP 파지 중심: odom 좌표, m
    current_base_xyyaw=[0.0, 0.0, 0.0],  # 같은 시각 odom->base_footprint, yaw rad
    model=model,
    policy=GraspPolicy(object_width_m=0.030),
    frame="odom",
)
goal = result["best"]  # None이면 현재 조건과 탐색 격자에 해가 없음
if goal is not None:
    print(goal["base_x_m"], goal["base_y_m"], goal["base_yaw_rad"])
```

RealSense optical-frame 좌표를 그대로 넣지 말고 인식 시각의 TF로 odom 프레임으로 변환해야 합니다.
객체/베이스 관측의 시각 일치, depth scale, 카메라 외부 보정, TCP와 실제 파지 중심 오프셋도 확인해야 합니다.
물체의 파지 폭 방향을 제한하려면 `desired_closing_yaw_world`를 rad로 전달합니다.
바나나 장축 자체가 아니라 **손가락이 서로 닫히는 방향**이며, 180° 대칭을 허용합니다.

알려진 장애물을 world-frame 축정렬 상자로 전달할 수 있습니다.

```python
obstacles = [{"center": [0.7, 0.3, 0.2], "size": [0.3, 0.2, 0.4]}]
result = plan_base_positions([.50, 0, .05], [0, 0, 0], model=model,
                             obstacles_world=obstacles)
```

CLI에서는 같은 JSON 배열을 `--obstacles boxes.json`으로 읽습니다. 상자는 후보 자세에서
베이스·팔과 검사되며, 표면 위 물체를 다루려면 실제 크기의 테이블/지지대도 상자로 제공해야 합니다.
**상자 정보를 제공해도 베이스 이동 경로와 현재 팔에서 사전 접근까지의 경로는 계획하지 않습니다.**
물체를 잡은 뒤 이 스크립트의 직선 인양 경로만 검사합니다. 장애물 정보가 없을 때
충돌 없는 이동을 가정하지 않으며 결과의 `navigation_collision_free`,
`navigation_path_checked`, `arm_from_current_to_pregrasp_checked`,
`continuous_collision_checked`, `contact_dynamics_validated`는 모두 `False`입니다.
오도메트리 이동기는 도착 후 새 TF와 물체 관측으로 다시 검증하고, 별도로 안전한 이동/팔 경로를 검증해야 합니다.

## 출처

공식 자산: https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/tree/humble

공식 설치/로봇 설명: https://docs.robotis.com/docs/systems/turtlebot3/manipulation/

Isaac API는 이 PC의 `~/IsaacLab/source/isaaclab`와 URDF 변환 예제에 맞춰 작성했습니다.
공식 자산의 Apache-2.0 라이선스는 `vendor/turtlebot3_manipulation/LICENSE`에 보존되어 있습니다.
