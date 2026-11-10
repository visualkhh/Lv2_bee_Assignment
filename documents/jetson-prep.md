# Jetson 준비 통합본 (2026-11-10)

## 기준과 포함 범위

이 준비 브랜치는 `develop`의 `b857e9c5` (#151 포함)를 기준으로 만들었다.

아직 `develop`에 병합되지 않은 PR의 현재 head를 모두 로컬 통합 브랜치에 반영했다.

- #149 place 릴리즈 간격 / MoveIt 시작 허용 오차
- #148 RL Nav2 heading 속도 제한
- #147 cuVSLAM map→odom 점프 gate
- #146 미션 방문 순서 파라미터
- #145 navigate 진척 감시
- #144 perception v5 모델 및 장기 기억 변경
- #141 경기장 벽 사전 지도와 막힌 접근면 제거
- #140 Coordinator 단계 timeout 및 실패 처리
- #97 ONNX Runtime 정책 실행

#141과 #146은 플래너 파일을 함께 바꿔 수동으로 통합했다. 두 기능과 테스트를 모두 보존하고, 함수/테스트 경계도 확인해 수정했다. 이 브랜치는 Jetson 검증을 위해 origin에 별도 이름으로 push한다. `develop`이나 원본 PR은 변경·병합하지 않는다.

`lv2_module6/tools/field-test.py`와 `lv2_module6/tools/README.md` 변경은 ROS bag을 기록하며 센서 확인, 제한된 저속 주행, Coordinator 단일 skill, 전체 미션을 단계적으로 실행하고 사전조건과 관측을 남기는 현장 준비 도구다. 단일 skill과 미션은 물리 로봇에서만 실행해야 한다. 새 bag이 기본이며, 비교 bag은 읽기 전용이고 `--attach-active`는 기존 recorder를 중지하지 않는다.

## 현재 검증과 제한

- Python 구문 검사 및 runner 도움말 출력 성공
- `git diff --check` 통과
- PR #141 + #146 통합 코드 경계 수동 확인
- ROS/colcon 테스트는 미실행: 이 Mac의 Docker daemon에 연결할 수 없어 프로젝트 CI 컨테이너를 시작하지 못했다.
- Jetson 빌드, 모델/가속기 호환성, 카메라·OpenCR·Nav2 동작, bag 기록은 아직 검증되지 않았다.

따라서 이 브랜치는 현장 배포 준비 후보이지, 실기 배포 완료본이 아니다. ROS 빌드와 테스트를 통과한 뒤에만 Jetson에 동기화한다.

## 현재 연결 상태

2026-11-10 확인 기준 Tailscale 기기 목록에 Jetson이 없다. 보이는 Linux 기기 `pa27-Legion-Pro-5-16IAX10` (`100.98.44.58`)는 학습용 PC이며 Jetson으로 취급하지 않는다. Mac의 Tailscale 상태도 coordination server에 동기화하지 못하고 있어, 지금은 원격 호스트의 신원을 확인하거나 배포할 수 없다.

## Jetson 연결 뒤 순서

1. Jetson이 Tailnet에 나타나면 hostname, OS/Ubuntu/ROS 배포판, 저장소 경로, 현재 branch/commit, 작업 트리 상태를 확인한다. 변경 파일이 있으면 보존하고, 확인 전에는 덮어쓰지 않는다.
2. 후보 브랜치를 Jetson이 지원하는 ROS 2 Lyrical 환경에서 빌드한다. CI의 `test-all` 및 관련 plan, skills, ml, perception, ros_system 테스트를 우선 실행한다.
3. Jetson의 CUDA/TensorRT/ONNX Runtime, 메모리 여유, 카메라 드라이버, OpenCR 연결을 확인한다. 모델 v5와 ONNX 정책 파일의 로딩 여부를 주행 없이 검사한다.
4. `tools/field-test.py run --label <단계>`로 시작해 각 단계에서 화면에 나온 로봇 위치, 퍽, 받침대/적재함 상태를 직접 확인·기록한다. 각 구동 동작은 별도 확인을 요구한다. 처음부터 전체 미션을 실행하지 말고 센서와 제한된 단일 동작부터 진행한다.
5. 로그와 bag을 검토하고 문제를 수정한 후에야 전체 미션 및 최종 실기 확인을 한다.

저장소 업데이트와 실제 로봇 구동은 Jetson 연결 및 빌드 준비 검증 후 수행한다. 브랜치 이름은 `prep/jetson-all-open-prs`다. Jetson의 기존 `develop` 체크아웃은 유지하고, 이 브랜치는 별도 worktree에서 빌드·검증한다.
