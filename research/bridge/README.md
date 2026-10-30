# 독립 ROS2 파지/베이스 계획 연결

배포와 실행은 [DEPLOYMENT.md](DEPLOYMENT.md)에 정리했습니다.

- `tb3_grasp_ros2/bridge.py`: 기본 dry-run ROS2 노드.
- `tb3_grasp_ros2/safety.py`: 단계/거리/회전 제한, LiDAR·odometry 안전 검사.
- `tb3_grasp_ros2/planner_adapter.py`: 분석 Planner와 실제 C++ 후보 IK/충돌 검사 연결.
- `native_probe.cpp`, `build_native.py`: 기존 헤더를 변경하지 않는 CPU 계산 adapter.
- `audit_planner.py`: Isaac 없이 의존성·좌표·URDF/cache·Workspace CSV 검증.
- `executor_contract.json`: Jetson에서 읽은 인터페이스/헤더/기존 파일 해시와 IK 차이.
- `results/planner_audit.json`, `jetson_runtime_audit.json`: 실제 노트북/Jetson CPU 검증.
- `results/unit_tests.log`, `mock_integration_report.json`: 하드웨어 없는 테스트 결과.
- `results/jetson_source_integrity.json`: 기존 Jetson 소스의 작업 후 해시.
- `build_bundle.py`: Mesh, Isaac, 개발환경을 제외한 Jetson 배포 묶음 생성.

실제 하드웨어 명령은 실행하지 않았고, 기존 Jetson 프로젝트 파일은 수정하지 않았습니다.
