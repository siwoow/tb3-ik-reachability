# TurtleBot3 + OpenMANIPULATOR-X: IK Reachability & Grasp Bridge

공개 시각화: **https://siwoow.github.io/tb3-ik-reachability/**

1,403개는 베이스 위치가 아닌 TCP 파지 중심의 저장 샘플입니다. 초록은 수직 접근·하강·닫기·인양의 기하학적 IK/FCL 검사 통과를 뜻하며, 물리 파지 성공이 아닙니다. 빨강은 당시 실제 바나나 후보 9개의 저장된 IK 실패입니다. 연구 코드가 추가되어도 Pages의 `index.html`과 Plotly 데이터는 유지됩니다.

## 소스 구성

- `research/analysis/`: 기존 4-DOF 모델, Planner, FCL, Workspace 계산, 기존 테스트, 로봇 메시·충돌 캐시 및 저장 결과.
- `research/analysis/results/reachability_1403_20261030/`: link1로 정렬한 기존 데이터, PNG 및 시각화 소스.
- `research/analysis/sim_collection/`, `sim_grasp_validation/`: 기존 Isaac Sim 실험 및 정지 Viewport 렌더 코드. 물리 시험은 파지 성공을 검증하지 못했고, 기구학 시연은 별도 모드입니다.
- `research/bridge/`: 노트북에 보관된 ROS2 Bridge, Visual Servoing, native adapter, 배포 문서, 기존 테스트 및 헤더 사본.
- `research/wheels/`: Jetson ARM64 / CPython 3.10용 기존 python-fcl wheel.
- `research/SOURCE_MANIFEST.json`: 원본 파일과 복사본의 SHA-256.

**이 저장소는 노트북에 저장된 소스의 스냅샷입니다. 현재 Jetson 파일을 가져온 것은 아닙니다.** Jetson에서만 수정한 최신 코드, 하드웨어 작업공간, YOLO/SAM 가중치, 실제 카메라 영상·Depth, 인증 정보, 가상환경, 로그와 빌드 산출물은 포함하지 않습니다. clone이나 설치만으로 로봇을 실행하지 않습니다.

## Jetson에서 clone 및 CPU 설치

Ubuntu 22.04 / ROS2 Humble / Python 3.10 / aarch64를 전제로 합니다. 기존 Bringup은 운영자가 관리합니다.

```bash
git clone https://github.com/siwoow/tb3-ik-reachability.git
cd tb3-ik-reachability/research/bridge
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=10
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install 'numpy==1.26.4'
.venv/bin/python -m pip install ../wheels/python_fcl-0.7.0.11-cp310-cp310-manylinux_2_24_aarch64.manylinux_2_28_aarch64.whl
.venv/bin/python audit_planner.py --planner-root ../analysis --output /tmp/tb3_planner_audit.json
.venv/bin/python build_native.py --source-root ~/turtlebot3_ws/src/pick_and_place
```

기존 헤더 해시가 계약과 다르면 native 빌드를 중단합니다. 실제 원본을 덮어쓰지 말고 차이를 먼저 확인하세요. Planner CPU 런타임은 NumPy/FCL과 URDF/캐시를 사용하며 Isaac Lab이나 Mesh 로딩이 필요하지 않습니다. URDF 내 메시 절대 경로와 캐시 해시는 원본 그대로 보존했습니다.

이미 정상 실행 중인 ROS2 서비스와 연결하는 별도 선택 명령입니다. 실제 이동·파지 옵션은 포함하지 않습니다.

```bash
.venv/bin/python -m tb3_grasp_ros2.bridge --dry-run \
  --planner-root ../analysis --native-probe ./native_probe
```

`visual_servo.py`는 기존 `cleanup_perception` Python 모듈, 카메라 보정 파일, YOLO/SAM 가중치, `native_probe_45` 및 별도 pick 실행기/서비스가 필요합니다. 이 파일들을 clone만으로 모두 제공한다고 가정하지 마세요. 기존 `DEPLOYMENT.md`는 최초 배포 시점 기록이므로 이 README의 clone 경로와 `--planner-root`를 우선 사용하세요. 노트북에서 성공한 검사가 Jetson 실물 호환성이나 파지 성공을 증명하지는 않습니다.

## Jetson에서 변경 후 push

공개 clone에는 인증이 필요 없습니다. push는 Jetson에서도 **본인 siwoow 계정**으로 인증해야 합니다. 노트북의 인증 파일이나 SSH 개인키를 복사하지 마세요.

GitHub CLI가 설치된 Jetson에서:

```bash
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
cd ~/tb3-ik-reachability
git config user.name siwoow
git config user.email 107081324+siwoow@users.noreply.github.com
git pull --ff-only
# 이 clone 안에서 수정하거나, 기존 Jetson 파일 중 필요한 소스만 해당 경로로 복사하세요.
git status --short
git diff -- research/
git add research/
git diff --cached --stat
git commit -m "Update research sources from Jetson"
git push origin main
```

기존 `~/tb3_grasp_bridge`나 `~/turtlebot3_ws`에서 수정한 파일은 이 clone에 자동 반영되지 않습니다. 최신 변경을 push하려면 필요한 소스만 `research/bridge/` 등으로 복사한 후 diff를 확인하세요. `.env`, 인증 파일, 영상, 가중치 및 로그를 강제로 추가하지 마세요.

## Isaac / 시각화 재현 범위

Isaac Sim과 렌더링은 노트북용이며 Jetson CPU 설치 대상이 아닙니다. 원래 실험 스크립트의 노트북 경로와 과거 실험 디렉터리 참조는 수정하지 않고 보관했습니다. 다른 PC에서 재실행하려면 설치 위치·메시 경로와 입력 경로를 별도로 지정/준비해야 합니다. 실제 RGB-D 캡처를 요구하는 시각화 전처리는 비공개 로컬 캡처가 필요합니다. 이미 저장된 1,403점/9후보 CSV·JSON·렌더 데이터와 공개 HTML은 그대로 확인할 수 있습니다.

원본 제어 코드와 기존 시스템 SDK/Bringup은 이 업로드 과정에서 변경하거나 실행하지 않았습니다. 제3자 라이선스는 `THIRD_PARTY_NOTICES.txt`, `licenses/Apache-2.0.txt` 및 각 원본 파일의 고지를 따릅니다.
