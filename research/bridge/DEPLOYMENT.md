# Jetson ROS2 Humble 연결

기존 `project_bringup`, `project.launch.py`, 하드웨어 설정, `grasp_trial_odom.py`,
`scan_move.py`, 실행기 C++ 코드는 수정하지 않습니다. 새 노드는 기존에 실행 중인
ROS2 시스템의 서비스/토픽만 이용합니다. 이 작업에서는 실제 이동·파지 서비스를 호출하지 않았습니다.

## 검증된 실행 환경과 파일 구조

Jetson ARM64 / Python 3.10에서 Isaac Lab 없이 Planner를 실제 실행했습니다.
Python-FCL은 시스템에 설치되어 있지 않아 ARM64 wheel을 임시 폴더에 풀어 사용했고,
검증 후 임시 폴더를 정리했습니다. 기존 Jetson Python 환경에 패키지를 설치하지 않았습니다.
노트북의 독립 Python 3.10 환경에서도 동일한 검증이 통과했습니다.

계획 실행에 필요한 외부 Python 패키지는 **NumPy와 python-fcl**입니다.
ROS 노드는 기존 Humble의 `rclpy`, `tf2_ros`, 메시지 패키지와 `cleanup_interfaces`도 사용합니다.
Isaac Lab, Isaac Sim, torch, scipy, matplotlib, trimesh는 런타임에 필요하지 않습니다.
검증 스크립트는 이 여섯 모듈의 import를 강제로 차단한 상태에서 계산을 재현했습니다.

배포 번들 구조:

```text
tb3_grasp_bridge/
  tb3_grasp_ros2/          # 별도 ROS 노드, 안전 검사, Planner adapter
  planner_runtime/
    tb3_grasp/            # 원본 분석용 Python 계산 모듈
    assets/resolved.urdf
    assets/collision_bounds.json
    results/grasp_workspace.csv
    results/workspace_summary.json
  wheels/                 # ARM64 CPython 3.10 FCL wheel
  native_probe.cpp         # 기존 헤더를 사용하는 순수 계산용 C++ adapter
  build_native.py
  executor_contract.json  # 실제 확인한 헤더/기존 파일의 SHA-256
  bundle_manifest.json
  audit_planner.py
```

Mesh·USD·Isaac 환경 전체는 배포하지 않습니다. 런타임 충돌 캐시에 볼록 껍질과 기본 형상,
구조적 충돌 제외 쌍이 들어 있습니다. URDF와 캐시의 해시가 다르면 실행을 중단합니다.
URDF의 Mesh 절대 경로는 노트북 경로지만 계획 런타임은 Mesh를 열지 않습니다.

`grasp_workspace.csv`는 1,403개 샘플이며 다음 열을 갖습니다.

```text
x_m,y_m,z_m,q1_rad,q2_rad,q3_rad,q4_rad,quality_score,joint_margin_score,floor_clearance_m,branch
```

좌표는 **base_footprint**, 단위는 m/rad입니다. 전체 저장 점의 FK 잔차와 관절 제한을 확인했습니다.
CSV를 실제 `reachable=true` 판정 대신 사용하지 않습니다. 실시간에는 `tb3_grasp/planner.py`를 호출합니다.

## 실제 실행기와의 차이

| 항목 | 분석용 Planner | 기존 Jetson 실행기 |
|---|---|---|
| 좌표계 | base_footprint | link1 |
| 입력 의미 | TCP 파지 중심 | 관측된 물체 표면 |
| 피치 | 수직 하향, Ry(+π/2) | 수평 기준 −80°~−15°, 2.5° 간격 |
| 전방 보정 | URDF 자체 | 실제 파라미터 `grasp_forward_offset`, 현재 launch 값 +30 mm |
| 관절 제한 | 원본 URDF + 추가 여유 | 더 엄격한 기존 제한과 0.025 rad 여유 |
| 접근/하강 | 수직 접근 60 mm | 표면 위 50 mm, 몸통 안쪽 깊이, 6 mm 이상 손가락 바닥 여유 |
| 길이/각도 | URDF에서 유도 | 기존 코드의 L2=0.13025 m, ALPHA2=0.1853 근사 |

두 IK를 동일하다고 가정하지 않습니다. 관측 좌표는 변경 없이 실제 평가 서비스에 보냅니다.
분석용에는 표면보다 10 mm 낮은 TCP 가설을 주고, 실제 전방 보정을 한 번만 적용해 후보를 만듭니다.
수직 접근 후보를 기반으로 최대 100 mm 뒤쪽 stand-off 후보도 검사합니다. 이 추가 후보는
수직 IK가 가능하다는 뜻이 아니라 **기존 사선 접근 IK로 검증할 후보**입니다.
이동 거리·회전량과 분석 점수로 정렬한 뒤, 현재 관절 자세에서 기존 C++ 후보 IK 및 추가
자기 충돌/바닥 검사를 통과하는 후보를 고릅니다. 전역 이동 경로가 안전하다는 보장은 하지 않습니다.

`native_probe`는 기존 `candidate_grasp.hpp`와 `grasp_kinematics.hpp`를 그대로 include합니다.
ROS나 하드웨어 기능이 없고 계산 결과만 JSON으로 출력합니다. 헤더 해시가 변경되었거나
실행 파일의 빌드 해시가 맞지 않으면 재감사 없이 실행하지 않습니다.

기존 `fingerPathClear`의 6 mm 검사와 엄격한 관절 제한은 그대로 유지합니다.
추가 FCL 검사는 현재 자세→pregrasp→하강→인양 경로를 관절 보간해 검사합니다.
사선 자세에서는 큰 OBB가 바닥을 과도하게 침범할 수 있어, 바닥 검사는 전체 볼록 형상의
정확한 최저 높이로 6 mm 여유를 확인합니다. 자기 충돌 Mesh와 구조적 제외 쌍은 분석 모델을 유지합니다.
추가 검사 때문에 기존 서비스가 reachable=true여도 파지 요청을 거부할 수 있습니다.

## 기존 기능 재사용

- `grasp_trial_odom.py`: 동일한 `/cleanup/capture_objects` 요청, 파지 후보 선택,
  `/cleanup/pick_target` 발행, `/execute_pick_and_place` 실행기 경로를 재사용합니다.
  기존 스크립트를 별도 프로세스로 실행하지 않으므로 중복 인식/임의 arm 명령이 생기지 않습니다.
- `scan_move.py`: `/odom` 진행 거리와 yaw 보정, 전진 ±25° 영역 45 cm,
  회전 전방위 30 cm 안전 기준을 유지합니다. 기본 이동은 45 mm, 하드 한계 50 mm이고
  회전은 최대 10°로 나눕니다. 기존 스크립트의 자동 30° 탐색 시퀀스는 실행하지 않습니다.
- `/cleanup/evaluate_grasp`: 매번 `require_candidate=true`. 실제 서비스가 reachable=true이면
  베이스 이동을 즉시 끝내고, 별도 `--execute-grasp`가 있을 때만 최종 재인식·평가 후 파지합니다.
  `approach_available`을 파지 승인으로 사용하지 않습니다.
  `candidate_approach` 관측은 이동 후보에만 사용하며, 실행에는 최종 `candidate_body` 관측도 요구합니다.

## 배포

노트북에서 준비된 압축 파일을 전송합니다. 아래 명령은 사용자가 실행할 배포 절차이며,
이번 작업에서는 기존 Jetson 프로젝트에 배포하거나 하드웨어를 실행하지 않았습니다.

```bash
scp ~/tb3_grasp_ros2/dist/tb3_grasp_bridge.tar.gz turtlebot:~/
ssh turtlebot
tar -xzf ~/tb3_grasp_bridge.tar.gz -C ~/
cd ~/tb3_grasp_bridge
```

Jetson의 별도 터미널에서 환경만 source합니다. 이 문서에서는 기존 launch를 새로 실행하지 않습니다.
기존 시스템과 같은 `ROS_DOMAIN_ID`를 사용하세요. 테스트용 도메인 187을 실제 연결에 사용하면 안 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# 기존 프로젝트와 격리된 Python 환경
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-deps wheels/python_fcl-0.7.0.11-cp310-cp310-manylinux_2_24_aarch64.manylinux_2_28_aarch64.whl

# 기존 원본 헤더는 읽기만 하고, 실행 파일은 이 폴더에 생성
.venv/bin/python build_native.py --source-root ~/turtlebot3_ws/src/pick_and_place
.venv/bin/python -I audit_planner.py --planner-root ./planner_runtime --output planner_audit.json
```

검증된 Jetson에는 NumPy가 이미 있습니다. `python3 -m venv` 자체가 없다면 Python 3.10용 venv 지원을
먼저 준비해야 합니다. 기존 프로젝트의 Python 환경에 패키지를 덮어쓰지 마세요.
이 wheel은 ARM64 Python 3.10용입니다. 다른 Python/아키텍처에서는 해당 버전용 wheel이 필요합니다.

기본 dry-run:

```bash
.venv/bin/python -m tb3_grasp_ros2.bridge --dry-run
```

실제 인식/평가 서비스는 호출하지만 `/cmd_vel`, pick target publisher와 파지 실행 client는 만들지 않습니다.
Capture 서비스는 기존 구현대로 인식 결과/증거 파일과 object registry를 갱신합니다.
하드웨어 이동은 없지만 다른 cleanup mission과 동시에 실행하지 마세요.

명시적으로 허용할 작업만 선택합니다. **아래 실행 옵션은 이번 작업에서 실제 로봇에 실행하지 않았습니다.**

```bash
# 베이스만 허용. 도달하면 멈추며 팔은 움직이지 않음
.venv/bin/python -m tb3_grasp_ros2.bridge --execute-motion --max-steps 12 --max-distance .30

# 베이스와 기존 파지 실행기 모두 허용
.venv/bin/python -m tb3_grasp_ros2.bridge --execute-motion --execute-grasp --max-steps 12 --max-distance .30

# 베이스 이동 없이, 이미 도달 가능한 물체에만 기존 파지 실행 허용
.venv/bin/python -m tb3_grasp_ros2.bridge --execute-grasp
```

`--dry-run`은 실행 플래그와 함께 쓸 수 없습니다. `--execute-motion`만으로 파지가 허용되지는 않습니다.
Nav2, 기존 `scan_move.py`, teleop 등 다른 `/cmd_vel` 발행자가 있으면 실제 이동을 거부합니다.
그 노드를 임의로 종료하거나 기존 launch/하드웨어 설정을 바꾸지 않습니다.

## 동작 및 제한

```text
정지 확인 → LiDAR → 새 인식 → timestamped TF로 odom 변환 → 실제 IK
  reachable=true: 이동 종료 → [명시적 허용 시] 최종 재인식/IK/충돌 확인 → 기존 파지 실행기
  reachable=false: Planner 후보 → 기존 C++ IK/FCL 검증 → 최대 45 mm 또는 10° 이동
                  → 반복 정지 → LiDAR → 새 인식 → 실제 IK 재평가
```

관측이 이미 odom이면 이중 변환하지 않습니다. 카메라/다른 world frame이면 **관측 시각의 TF**를 사용하며
최신 TF로 임의 대체하지 않습니다. 기존 perception도 실제 timestamped camera/wrist TF와 보정을 사용합니다.
base_link와 base_footprint 사이의 10 mm 차이도 TF로 처리합니다. 실제 arm mount가 모델과 3 mm 이상
다르면 이동을 거부합니다. 모델은 평평한 floor z=0용이며 경사진 지면이나 다른 floor 설정은 지원하지 않습니다.

기본 속도 0.020 m/s, 각속도 최대 0.10 rad/s, 명령 약 20 Hz입니다. 총 이동 0.30 m,
총 회전 90°, 최대 12단계, 전체 동작 180초로 제한합니다. CLI로 단계 수와 총 거리를 줄일 수 있으며
하드 상한은 20단계/0.50 m입니다. 모든 회전도 단계 수에 포함합니다.
명령한 한 단계는 45 mm이며, 측정 이동이 50 mm를 넘거나 정지 후 초과가 확인되면 고장으로 잠급니다.
실물의 관성·오도메트리 오차 때문에 물리적으로 정확히 50 mm 이내에 멈춘다고 보증하지 않습니다.

정지는 반복 zero Twist, odometry의 실제 정지 상태 0.5초 확인, 이후 재인식 순서입니다.
odom 0.30초, LiDAR 0.50초, joint feedback 0.25초, 명령 heartbeat 0.15초를 검사합니다.
메시지 수신 시각과 원본 timestamp를 모두 검사합니다. 새 관측은 마지막 정지 이후의 이미지여야 합니다.
베이스 이동 중 팔이 움직이거나 관찰/파지 중 베이스가 움직이면 중단합니다.
장애물·센서 결손·TF 결손·평가 실패·타임아웃·횟수/거리 초과는 모두 정지 후 자동 재시도하지 않습니다.

원본 두께나 마찰, 미끄러짐을 성공 확률로 계산하지 않습니다. 새로운 표면 중심은 기준점에서
40 mm 이내여야 하며, 후보 banana는 하나여야 합니다. 신뢰도 0.5 이상, 위치 불확실성 30 mm 이하,
관측 나이 2초 이하를 요구합니다. 오래 걸리는 인식은 안전하게 거부될 수 있습니다.

2D LiDAR만으로 테이블 상판·낙차·팔 높이의 장애물을 보증할 수 없습니다. 경로 충돌 보장은 없습니다.
통신 장애 시 베이스에 zero Twist를 보내지만, 네트워크 단절/프로세스 강제 종료 때 실제 제동까지
보장하려면 기존 드라이버의 cmd_vel timeout과 하드웨어 정지 수단이 필요합니다. 이 설정은 변경하지 않습니다.
기존 파지 서비스 Trigger에는 취소 프로토콜이 없습니다. 파지 요청 이후 실패하면 base stop과 arm/gripper
action 취소를 best effort로 요청하고 재시도하지 않습니다. 연결이 끊긴 팔의 정지를 보증하지 않습니다.
파지 안전의 최종 책임은 기존 실행기의 fresh-feedback·관절 제한·바닥 검사·controller feedback에 남아 있습니다.

## 결과와 테스트

`results/bridge_events.jsonl`에는 관측 좌표, 실제 IK 응답, Planner 후보, 실행할 한 단계,
정지·실패 원인과 실제 누적 거리/회전이 저장됩니다. 비밀번호나 인증 정보는 저장하지 않습니다.

노트북 검증:

```bash
cd ~/tb3_grasp_ros2
.venv/bin/python -m unittest discover -s test -v
source /opt/ros/humble/setup.bash
source ./test_ws/install/setup.bash
.venv/bin/python test/run_mock_tests.py
```

ROS 통합 테스트는 `ROS_LOCALHOST_ONLY=1`, 별도 domain 187, `/bridge_test/*` 토픽만 사용합니다.
dry-run 무발행, reachable 즉시 정지, timestamped 카메라 TF, 이동→정지→재인식→실제 IK→모의 파지,
LiDAR/odom 단절 정지, 평가 서비스 실패를 검사합니다. 모의 파지는 단순 테스트 응답입니다.
실제 `/cmd_vel`, `/cleanup/pick_target`, `/execute_pick_and_place`는 검증 중 호출하지 않았습니다.
