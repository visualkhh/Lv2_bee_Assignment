# Jetson 준비 통합본 (2026-11-10)

## 통합 범위

`develop`의 `b857e9c5` (#151 포함)를 기준으로 미병합 PR의 현재 head 9개를 `prep/jetson-all-open-prs`에 통합했다.

- #149 place 릴리즈 간격 / MoveIt 시작 허용 오차
- #148 RL Nav2 heading 속도 제한
- #147 cuVSLAM map→odom 점프 gate
- #146 미션 방문 순서 파라미터
- #145 navigate 진척 감시
- #144 perception v5 모델 및 장기 기억 변경
- #141 경기장 벽 사전 지도와 막힌 접근면 제거
- #140 Coordinator 단계 timeout 및 실패 처리
- #97 ONNX Runtime 정책 실행

#141과 #146은 같은 플래너 파일을 수정해 충돌이 났다. 두 기능과 테스트를 함께 보존하도록 통합했다. 원본 PR과 `develop`은 수정·병합하지 않았다. 후보 브랜치는 Jetson 검증을 위해 origin에 push했다.

`lv2_module6/tools/field-test.py`는 ROS bag 기록과 함께 센서 확인, 제한된 저속 주행, Coordinator 단일 skill, 전체 미션을 단계별로 실행하고 사전조건과 관측값을 남긴다. 기존 bag은 읽기 전용 비교 기준이며 `--attach-active`는 현재 recorder를 중지하지 않는다.

## Jetson 환경과 경로

저장소에 기록된 SSH 대상은 `bee@bee.local`이며, 이번 세션에서는 `10.2.12.110`으로 접속해 `hostname=bee`, Ubuntu 24.04, JetPack R39, ARM64, ROS 2 Lyrical을 확인했다.

사용자가 지정한 위치는 `/home/ghj`다. 기존 `/home/ghj/Lv2_bee_Assignment`에는 `feature/cuvslam`의 미커밋 변경과 생성된 파일이 있다. 사용자는 해당 이전 작업을 제거해도 된다고 승인했고, 최신 후보를 우선 사용하도록 요청했다. 별도 후보 worktree는 `/home/ghj/Lv2_bee_Assignment-jetson-prep`에 있다. 기존 checkout을 최신 브랜치로 교체한 후 실제 사용할 경로에서 다시 빌드한다.

## Jetson 검증 결과

- ROS 2 워크스페이스 전체 9개 패키지 빌드 성공. `ML_ONNXRUNTIME_ROOT=/opt/onnxruntime`로 Jetson의 ONNX Runtime을 연결했다.
- `plan`, `skills`, `ml`, `perception`, `ros_system` 테스트: 184 passed, 0 failed, 1 skipped.
- perception의 OpenCV marker 테스트는 OpenCV 4.6 `drawMarker`와 4.7+ `generateImageMarker`를 모두 지원하도록 수정했다.
- ML 정책 테스트가 YOLO detector를 제어 정책으로 잘못 읽던 점을 수정했다. 이제 plan/nav/skills 정책만 확인한다.
- field runner의 Python 구문 검사·도움말과 `ros2 bag record --help` 확인 성공.
- TF2 헤더와 ament-index API deprecation 경고가 남아 있지만 빌드 차단 오류는 아니다.
- 실기 bringup, 바퀴·팔·그리퍼 동작, 카메라 스트림, rosbag 실기 기록은 실행하지 않았다.

실기 확인은 최신 checkout의 빌드와 자동 테스트를 마친 뒤 field runner로 한 단계씩 진행한다. 물리 동작은 실행하지 않았다.

## 2026-11-11 최종 재확인

- Jetson은 `bee` (`10.2.12.110`)로 재접속됐고 `/home/ghj/Lv2_bee_Assignment`에서 최신 후보 브랜치의 문서 업데이트까지 fast-forward했다.
- 실제 지정 경로에서 `plan`, `skills`, `ml`, `perception`, `ros_system` 테스트를 다시 실행해 184 passed, 0 failed, 1 skipped를 확인했다.
- 최신 `develop` (`b857e9c5`)의 RL Nav Controller 모델(모델 커밋 `6b395c07`)과 후보 브랜치의 `policy.onnx` 및 `policy.onnx.data` blob이 일치한다. Jetson의 `install/ml/share/ml/model/rl_nav_controller/` 설치 파일 SHA-256도 저장소 원본과 일치한다.
- field runner Python 구문, 실행 도움말, `ros2 bag record --help` 확인 성공.
- 전체 9개 패키지 빌드 성공. deprecation 및 OpenCV 4.6/4.8 링크 경고는 남아 있다.
- bringup, 바퀴·팔·그리퍼 동작, 카메라 스트림, 실기 rosbag 기록은 실행하지 않았다.
