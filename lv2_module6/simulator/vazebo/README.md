# SIL 시뮬레이터

실기 대신 세상(코스·퍽·장애물·이동물체·카메라 장면)과 로봇 몸(OpenCR)을 흉내 내고, ROS 쪽은 실제 노드가 그대로 돈다. 호스트에서 띄우는 창이고, 컨테이너와는 `../../debug` 파일로만 주고받는다 (호스트에 ROS 불필요). 전체 개발 흐름은 [DEVELOP.md](gazebo/DEVELOP.md).

```bash
# 컨테이너 (저장소 루트에서 — 둘 중 하나로 띄워 두기)
cd docker && docker compose up -d && docker compose exec lyrical bash   # 컨테이너 셸
test-faker 0                          # 기본 — 컨테이너 fake_opencr가 OpenCR 역할 (미끄럼 · 마찰 · 충돌 · 통신 끊김, 시뮬레이터 버튼으로 요청)
test-faker 0 mock_hardware:=true      # OpenCR 없이 mock: 바퀴·팔은 명령 그대로 (물리 없음)
# 호스트 (새 터미널)
cd lv2_module6/simulator
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회
.venv/bin/python main.py
.venv/bin/python main.py --snapshot out.png                # 창 없이 한 장 (확인용)
```

| 시뮬레이터 기능 | 필요한 컨테이너 명령 | 안 맞으면 |
| --- | --- | --- |
| 카메라·명령·수동 조작·service/action 입력·상태/진단 보기 | `test-faker 0` (mock도 됨) | 카메라 `waiting`, 진단은 `(진단 없음)` |
| 미끄럼·마찰·충돌·통신 끊김·팔 붙잡기 (fake_opencr) | `test-faker 0` (기본) | `mock_hardware:=true`면 상태 'OpenCR 없음', 실제 로봇(채움) 안 뜸 |

## 호스트 CLI (`bee.py`) — 컨테이너에 안 들어가고 지시

```bash
.venv/bin/python bee.py user "빨간 퍽을 목표 구역으로 옮겨"   # 텍스트 명령
.venv/bin/python bee.py status                                # 작업·모드·위치·service/action 목록
.venv/bin/python bee.py estop on|off                          # 비상정지
.venv/bin/python bee.py manual on|off                         # 수동 모드
.venv/bin/python bee.py twist 0.1 0.0                         # 수동 바퀴 (수동 모드에서만)
.venv/bin/python bee.py service score_grasps '{"object_id": 3}'   # 응답 대기, exit 2=호출 실패
.venv/bin/python bee.py action navigate_to_pose '{"x": 1.0, "y": 0.5}'  # accept→result 대기
.venv/bin/python bee.py action navigate_to_pose '{...}' --no-wait     # accept까지만
.venv/bin/python bee.py cancel navigate_to_pose <goal_id>
.venv/bin/python bee.py snap out.png
```

service/action은 `request_id`·`goal_id` 일치로 자기 응답만 받는다 (stale 무시). 창(`main.py`)을 안 띄우고 명령·호출만 할 때 쓴다.

| 화면 | 내용 |
| --- | --- |
| 지도 | 코스 + **실제 로봇(채움, fake_opencr 진짜 위치)** · `/odom`(테두리 — 미끄럼만큼 어긋남) · `base_footprint`(초록점, `/tf` — `/odom`까지 점선·거리) · 경로(`/plan` — Nav2 전역 경로, 주행 중 1초마다 새로 옴 · 3초 넘게 안 바뀌면 주행 끝난 것으로 보고 안 그림) · **[Nav2 지도] 체크**: Nav2가 실제로 보는 지도 `/map`(RTAB-Map 발행 — monitor_manager `topic/map/data`)을 벽 진회색 · 빈 곳 연파랑으로 겹침 (바뀔 때만 발행되므로 오래돼도 그림) · **[코스트맵] 체크**: `/global_costmap/costmap` 위에 `/local_costmap/costmap`을 RViz 색(100 자홍 · 99 하늘 · 1~98 파랑→빨강 = 벽과의 간격)으로 · **[점구름] 체크**: 로봇이 보는 장애물 점(LiDAR 대신 깊이 카메라 — `pole_cam/depth/color/points_filtered`, body_filter가 몸 뺀 것)을 바닥 높이 2~60cm만 빨간 점 + `gripper_cam/depth/color/points`(원본 — 로봇 중심 0.22m 안은 뺌, 지도 만들 땐 스캔 자세로 왼쪽 뒤)를 분홍 점으로 로봇 자리에 겹침 (monitor_manager `topic/<토픽>/data` 파일 — 실기에 붙여도 그대로) · 인지 물체(**로봇이 아는 위치 기준이라 위치추정 오차만큼 통째로 밀려 그려짐** — [감지 물체 보기] 창에 오차 표시, 3D 뷰는 로봇이 아는 위치로 옮겨 인지 오차만 보임 · 지도 드래그 순간이동은 EKF `/set_pose`도 같이 · `/tracked_objects` = 두 카메라 융합 결과, 보라 테두리 + 2σ 타원 · 두 카메라가 모두 붙은 트랙은 라벨에 **F**) · 융합 전 카메라별 검출(`/perception/<cam>/detections`, pole_cam 주황 · gripper_cam 분홍 ✕ + 2σ 타원 → 붙은 융합 트랙까지 선 — 확대하거나 [감지 물체 보기] 창에서) · 퍽(정답) · 바닥 장애물(회색) · 이동물체(파랑) · 두 카메라 시야. 퍽·장애물은 왼쪽 드래그로 이동, 로봇은 잡아끌어 순간이동(fake_opencr 즉시·mock 릴리스 때), 빈 곳 왼쪽 드래그 지도 이동, 휠 확대(커서 고정) · 오른쪽 드래그 이동 · [보기 리셋](확대·이동만 처음으로 — 지도 데이터·토픽과 무관) |
| 감지 물체 보기 | 왼쪽 아래 **[감지 물체 보기]** → 새 창 (`objects_window.py`, 0.5초 갱신, 누를 때마다 독립 창). 표: 융합 트랙(`/tracked_objects`) 한 줄 + 붙은 카메라별 검출(`/perception/<cam>/detections`) 하위 줄 — 위치 · σ x·y·z(공분산) · 융합 위치와의 차이 · 크기 · 신뢰도. 두 카메라가 모두 붙은 트랙은 id 옆 **F**. 확대도: 고른 트랙의 2σ 타원(융합 보라 · pole_cam 주황 · gripper_cam 분홍)을 cm 격자로 크게 — 지도에선 몇 픽셀. 아래에 두 카메라 ROS 영상 + 인지 박스(RGB-SEG와 같은 그림) |
| 상태 · 진단 | 오른쪽 열 [상태 · 진단] 탭 — 위 상태판(작업 · 계획 · Nav2 관리자 · `재시작 (state_manager) /state/restart` 등) · 아래 **진단 = 노드 전부**: 그래프의 노드마다 진단(`/diagnostics_agg` — 노드 항목 중 가장 나쁜 수준 + 메시지, 없으면 `(진단 없음)`). `/state/restartable`(state_manager)에 있는 노드는 밑줄 + `[재시작 pid …]` — **누르면 확인 창 → `/cmd/restart node <이름>`** (그 프로세스만 같은 명령줄·파라미터로, Nav2 수명주기 노드는 다시 활성화 필요 안내) · 컨트롤러 · costmap 하위 · 내부 노드는 `(프로세스 아님)` · Nav2 관리자가 멈췄으면 `[ROS 재시작] (전체)` 줄 |
| 로봇 3D | 지금 상태(URDF+관절값, 드래그 회전 · Shift+드래그 이동 · 휠 줌) + 주변 퍽·장애물·이동물체 — 바퀴 회오리(스포크 4개 — 바퀴 관절각만큼 돌아 굴러가는 게 보임, 주황=회전 중) + map·odom·base_footprint 좌표 + `관절축 보기` 체크(X빨·Y초·Z파, 바퀴는 같이 돌아 회전 표시) + **`충돌 모양 보기 (collision)` 체크** — 켜면 로봇 3D와 pole_cam·gripper_cam 렌더가 URDF `<collision>` 모양(주황)으로 바뀜 (메시 충돌체도 메시 그대로). 끄면 원래대로 + **`감지 물체` 체크** — 인지 결과를 3D에 겹침: `/tracked_objects` = 보라 와이어 상자(size 크기, 라벨 `class#id`, 두 카메라 융합이면 **F**) · `/perception/<cam>/detections` = 카메라 색 점(pole_cam 주황 · gripper_cam 분홍) → 붙은 트랙까지 선 (면이 아니라 선이라 실제 물체 모형을 안 가림, 조작 창 3D와 체크 공유) — 수동 조작 창의 로봇 뷰를 여기로 옮김 |
| 퍽 · 배경 · 바닥 | [퍽 랜덤 배치] 개수 · 모양(원기둥·상자·구) · 색 무작위 / 360° 파노라마 배경 / 바닥 텍스처(`images/floor`, 1타일=1m 반복, 없으면 격자) / 벽 텍스처(`images`, 가로 0.5m 반복, 없으면 단색) / [카메라 송출] 끄기 = 카메라 빠짐 |
| 환경 | 미끄럼(바퀴가 돈 만큼 덜 나감) · 마찰(가속 한계) · [이동물체 추가/삭제](미로 안 칸 사이를 돌아다님, 삭제는 나중 것부터) · [바닥 장애물 추가/삭제](삭제는 나중 것부터) · [신호등 추가/삭제](랜덤 배치, 평소 꺼짐 — 초록 5초 → 황 2초 → 빨강 5초, 사이 1초씩 꺼짐, 높이 벽 수준) · [로봇 위치 초기화](fake_opencr 위치 + odom 리셋) · OpenCR 통신 끊기 · 팔 붙잡기(과전류) · 조명(카메라 밝기 배율) · 카메라 잡음 |
| 미션 | `../../../documents/mission/MISSION.md` 방식 — 한 맵에 **미션 지역 여러 곳**(주황 판 · QR 판 · 퍽), 끝나면 출발 칸 복귀. 처음엔 빈 맵. **지도 오른쪽 클릭 메뉴**: 빈 칸 = [여기에 미션 지역 ▸ 문제] · [B 지역을 여기로] · [로봇을 여기로](진짜 위치 순간이동 — /cmd/fake_opencr/set_pose, 실기로 치면 들어 옮김) · [여기로 이동](Nav2) · **[여기가 로봇 위치 (초기 위치 지정)]**(위치추정 모드에서만 — RTAB-Map이 매핑 중엔 무시. `/initialpose` PoseWithCovarianceStamped → RTAB-Map에 '로봇은 여기', 방향은 지금 실제 로봇 방향, RViz 2D Pose Estimate와 같음. map 좌표는 고정 기준 = 출발 칸 원점·0° — 지도는 출발 칸에서 시작해 만듦. 지도 칸 **[실제 위치로 맞추기]** = 클릭 없이 실제 위치로), 미션 지역 위 = [문제 바꾸기 ▸] · [QR 판 돌리기] · [미션 지역 지우기] (가장 가까운 칸에 M1·M2…, 출발 칸 · B 칸은 안 됨). **Shift+왼쪽 드래그**로 미션 지역(판·QR 판·퍽 통째로)·B 지역 옮기기 — 놓으면 칸에 맞춰 붙고, 안 되는 칸이면 원위치. 판 위에 퍽·로봇이 있어도 통째로 잡힘. 그냥 드래그는 퍽·로봇·장애물 하나. 오른쪽 드래그 = 지도 이동. 퍽: 정렬 6개 3색 · 세우기 5개 중 1~3개 누움 · 옮기기 4개. QR 판은 벽 쪽에 붙어 통로 쪽을 봄(자동) — 옮긴 칸에서 벽 방향이 바뀌면 다시 맞춤. [환경 배치]: 바닥 장애물 3 · 이동물체 1 · 미끄러운 구간 2(하늘색, 카메라엔 안 보임) · 조명 변화 (seed 같으면 같은 배치), [환경 해제]. 시작 신호는 명령 줄의 [모든 미션 수행 후 출발 칸으로 복귀]. QR 무늬는 출력 해상도에서 원근 투영으로 입혀 0.4~1.0m에서 읽힘(perception → `/mission_qr`). 퍽 세우기: 집을 때보다 손을 60° 넘게 기울여 놓으면 누움 ↔ 섬. 문제 = `simulator/qr/<코드>/` 폴더 (폴더 이름 = 코드·지도 표시): `qr.png`(판에 입힐 QR 그림 — 대회 QR로 교체 가능) + `info.json`(`text` QR 텍스트 · `name` · `pucks` · `colors` · `lying`). 문제 추가는 폴더 하나, 시작 때 qr.png 해독값과 `text`가 다르면 경고. 환경 수는 `scene.py` `ENV_LAYOUT`. 채점은 외부(대회 심판) |
| 지도 (SLAM) | 컨테이너를 `test-faker 0 map:=slam`(스캔) 또는 `map:=saved`(저장 지도)로 띄웠을 때 RTAB-Map 상태(스캔 중 · 위치추정 · `/map` 갱신) + **[스캔 시작]**(고름: 이어서 스캔 = 지금 지도에 덧붙임 · 지우고 처음부터 = 같은 폴더 비움 `/rtabmap/rtabmap/reset` · 새 지도로 = 지금 지도 두고 새 폴더 `map:=slam` 재시작) · **[위치추정으로]**(`/rtabmap/rtabmap/set_mode_mapping` · `set_mode_localization`, 재시작 없이) + **[지도 저장]**(`/map_saver/save_map` → 지금 지도 폴더 `ros2_ws/map/<이름>/map.pgm`·`map.yaml` — 폴더 이름은 state_manager `/state/map_name`). 지도 한 벌 = 폴더 하나 `ros2_ws/map/<날짜_시각>/`(`rtabmap.db` · `map.pgm` · `map.yaml`) — `map:=slam`이 새 폴더를 만들어 덮어쓰지 않음, 지도 위 지도 목록(= state_manager `/state/map_list`, 로봇에 있는 지도 — 실기도 같음. 선택은 `/state/map_name`을 따라감)에서 폴더를 고르면 **그 지도로 갈아끼움**(`/cmd/map_name` → state_manager → RTAB-Map `load_database` + 위치추정 → `/map` · `map→odom`이 그 지도로) · [Nav2 지도]가 켜져 토픽 `/map`으로 보임. 호스트의 `ros2_ws/map` 파일은 읽지 않는다 (실기에 붙이면 의미 없음 — 화면의 ROS 정보는 전부 토픽). 위치추정은 `rtabmap.db`가 있어야 함(pgm은 미리보기). 스캔: [수동 조작] 탭으로 몰고 다님, 깊이는 pole_cam depth(시뮬레이터 렌더) |
| 수동 조작 | 우측 [수동 조작] 탭 (`manual_window.py`, 고르면 수동 모드 · 나가면 바퀴 정지 + 자동 모드): 바퀴 버튼 앞 · 뒤 · 멈춤 · 좌측 바퀴 앞/뒤 · 우측 바퀴 앞/뒤 · 전체 속도 ▲▼, 비상정지(mock 모드 벽 앞 자동 정지), **팔 3D**(몸체는 어둡게 · 팔은 회색으로 구분 + 주변 퍽·장애물·이동물체를 메인 창 로봇 3D처럼 로봇 기준으로 같이 · 바퀴 회전 스포크(바퀴 버튼·단축키로 움직이면 굴러가는 게 보임, 주황=회전 중) · `감지 물체` 체크(메인 로봇 3D와 같은 그림·같은 체크) · 노란 관절 마커를 잡아 위아래로 끌면 그 관절 회전 → 놓으면 `/cmd/manual/arm`, 목표 밝게 · 지금 어둡게, **손끝 목표** — 지금 손끝 손잡이(하늘색 점·원)를 **잡고 끌기** = 손끝 목표가 지금 손끝 자리에서 시작해 커서를 따라감 — **팔이 닿는 곳까지만**(URDF FK로 손끝이 아래를 보는 자세의 닿는 범위를 계산(모델이 오면 약 2초, 백그라운드) · MoveIt 실측 보정: 바깥 경계 2cm 깎음 · 팔 회전축 0.08m 안쪽(몸과 부딪힘)·joint1은 360°라 방향 제한 없음 · 로봇 몸체(차체·적재함·폴·바퀴 — URDF 충돌 모양을 3D 칸으로, 그리퍼 반폭 0.04m 넓힘)와 손끝~손끝 위 0.10m(손목) 높이가 겹치면 제외 — 적재함 위는 닿음, 그 위 pole 카메라는 손 위라 상관없음 — 밖으로 끌면 가장 가까운 닿는 곳에 멈춤 · 범위는 3D에 **지금 팔이 향한 방향(joint1 실측)의 수직 단면**으로 옅은 청록 면 — 팔이 돌면 같이 돔, 끄는 중엔 그 높이에서 팔을 돌려 닿는 고리(360°, 몸체에 막힌 곳은 끊김)를 진하게)(수평 · Shift면 높이, 끄는 도중에 Shift를 눌러도 바뀜 — MoveIt RViz 손끝 마커처럼, 하늘색 원 + 좌표축 + 바닥까지 세로선, 좌표 base_footprint는 팔 조작 열에 표시) · **닿는 범위 두 개**: 지금 모드 범위는 채움 · 다른 모드 범위는 외곽선 (라벨 `집기 자세 (손가락 아래)` · `전체 (방향 자유)`) — 팔을 든 자세처럼 손끝이 집기 범위 밖이면 잡아도 목표가 튀지 않고 그 자리에서 시작, 범위에 들어온 뒤부터 묶임 + "집기 자세로는 못 가는 곳" 경고 · **`손끝 방향 자유` 체크** = 방향 상관없이 위치만: 시뮬레이터가 FK 표 + 감쇠 최소제곱으로 IK를 풀어(joint5는 지금 값) MoveIt `/move_action` **관절 목표**로 (몸체와 안 부딪히는 경로, pick_ik는 위치만 준 목표를 잘 못 풀어서) — 못 닿으면 "IK 해 없음" · **손가락 방향** 체크 + 0~179°(손목 회전 joint5 — 끄면 자유, 켜면 손가락이 벌어지는 선의 수평 방위로 고정 ±0.05rad, 3D에 파란 선으로 표시 · 누운 퍽 등) → **[end_effector 이동]** = 수동 모드로 바꾸고 MoveIt `/move_action`(service_action_input 경유)으로 손끝을 그 자리에 위→아래 방향으로 계획+실행, 결과(성공 · 실패 이유 · 팔 받침에서 거리) 표시 · [이동 취소] · [목표 지우기] (확인 `test_manual_goal.py`), 팔/그리퍼 초기화, 그리퍼 열기/닫기, **[모터 재부팅]**(`/cmd/manual/reboot_motors` — 과부하 등 Hardware Error 난 DYNAMIXEL만, 에러가 있으면 버튼 아래 빨간 글씨 `모터 에러: joint2:과부하` — `/state/devices/<wheels·arm·gripper>`) · Shift+드래그 시점 이동) + **팔 조작 열**(오른쪽 세로 스크롤 — 팔·그리퍼 버튼 + 관절 슬라이더, joint1~5 °·gripper mm, URDF 한계 — 3D 마커와 같은 목표, 놓으면 **MoveIt `/move_action` 관절 목표**(몸체·폴과 안 부딪히는 경로, 불가능하면 거절하고 목표를 지금 자세로 되돌림 · MoveIt 이동이 성공하면 슬라이더·밝은 팔(목표)을 옮겨진 실제 자세로 맞춤 · 슬라이더는 놓거나 0.4초 멈추면 보냄) · '관절 직접 보내기' 체크 = 예전처럼 `/cmd/manual/arm`(충돌 검사 없음) · 그리퍼는 늘 직접 · **[퍽 집기]**(손끝 목표 = 집을 자리: 벌리기 → 위 0.05 → 내려가기 → 닫기 → 들어올리기) · **[내려놓기]** · **적재함 칸 1~4 [싣기]·[꺼내기]**(칸 = coordinator `tray_slots`, 높이 0.22·0.17 — coordinator place·pick tray와 같은 순서), 하나라도 실패하면 멈추고 이유 표시) + 바퀴 단축키(`W/X` 앞뒤 · `S` 멈춤 · `Q/Z` 좌바퀴앞뒤 · `E/C` 우바퀴앞뒤 · `A/D` 회전 · `[`/`]` 속도, 1.5초 자동정지). 로봇 전체 3D(지금 상태)는 메인 창에 있음 |
| 시리얼 (장비) | 로봇 3D 옆. 가상 시리얼 파일 그대로 — 위: SBC → OpenCR `C` 명령(`debug/output/hardware/serial`, 형식 틀린 줄도 그대로), 아래: OpenCR → SBC `S` 상태(`debug/input/hardware/serial`) 최근 7줄 · 읽은 시각(ms). 파일에 새 줄이 붙을 때만 갱신 — 시뮬레이터는 읽기만 한다 (S는 fake_opencr가 씀). 머리줄에 연결 상태(대기 = mock_hardware:=true라 시리얼 안 씀 · 연결 · 통신 끊김)와 C·S 초당 개수. 프로토콜: `firmware/README.md` |
| 카메라 2대 | 세로 스크롤 칸에 두 구역 — **HARDWARE**(시뮬레이터 렌더 = 카메라 원본) · **ROS**(받아 온 토픽, 실기도 같음). 구역마다 열 = pole_cam · gripper_cam, 줄 = RGB · RGBD · RGB-SEG(인지 물체 겹침) · POINT(드래그 회전, ROS는 monitor_manager `data` — base_footprint 기준 높이 색). 두 구역을 맞대 보면 렌더 → fake_camera → 인지·점구름 중 어디서 틀어졌는지 보임 (실기에 붙이면 HARDWARE는 비고 ROS만). 상태 · 진단은 창 맨 오른쪽 열 · 깊이는 실기 RealSense처럼 거리 제곱 잡음(σ = 0.005·z² m) · 물체 경계 구멍(이웃과 5% 넘게 튀면 0) · 결측 0.3% (`scene.realsense_depth`, 렌더 직후 — 컬러는 조명·잡음 슬라이더) |
| 자동 탐사 | '지도 (SLAM)' 칸 [자동 탐사] → `/cmd/operation_mode` EXPLORE (매핑 중에만, 코스 지도면 `map:=slam` 재시작 확인) → explorer가 프론티어로 Nav2를 차례로 보냄 · [탐사 멈춤] = AUTO · 수동 조작을 만지면 바로 멈춤 · 상태판 `탐사 결과 /explore/result` |
| 코스 지도에서 스캔·지도 고르기 | 컨테이너가 `map:=course`(RTAB-Map 없음)면 [스캔 시작] → `map:=slam`, [위치추정으로] → `map:=saved`, 지도 목록에서 고르기 → `map:=saved map_name:=<폴더>` 로 **ROS를 다시 띄울지 확인 창** → 예 = `/cmd/restart` |
| ROS 재시작 | 상태판의 **Nav2 바퀴 체인 · Nav2 주행 계획** 줄(`/lifecycle_manager_base|navigation/managed_nodes_activated`)이 빨간 '꺼짐'이면 — 기동 때 가끔(약 4번에 1번) Nav2 관리자가 서버 응답을 못 받고 멈춤 — 왼쪽 버튼 열 맨 아래 [ROS 재시작] (확인 창 → `/cmd/restart all` → state_manager가 끄고 빌드 후 같은 인자로 다시 — 결과는 상태판 `재시작 (state_manager) /state/restart`). 진단 칸에도 `Nav2 관리자` 묶음(bringup `diagnostics.yaml` nav2 — `lifecycle_manager_*: Nav2 Health`, 안 오면 Stale)으로 나오고, Nav2는 멈춰도 수준을 OK로 내므로 메시지가 `Managed nodes are active`가 아니면 시뮬레이터가 ERROR로 칠한다. 상태판·진단 칸의 빨간 밑줄 줄 **'여기를 눌러 [ROS 재시작]'** 을 누르면 같은 확인 창 → `/cmd/restart` → 컨테이너 `test-faker 0`이 ROS 전부 정상 종료 → 같은 인자로 다시 띄움 (1~2분). ROS가 막혀도 동작 — 받는 쪽이 ROS 노드가 아니라 test-faker 스크립트 |
| 로봇 모델 | [ROS 재시작] 위 선택 상자 — `../../urdf` 아래 `urdf.xacro`가 있는 폴더 목록(시뮬레이터를 켤 때 읽음, 컨테이너 `/ws/urdf`와 같은 폴더). 지금 모델은 `/robot_description` 메시 경로(`package://ros_system/urdf/<폴더>/meshes`)의 폴더 이름 — 메시 없는 모델은 URDF에 폴더 이름이 안 남아 마지막으로 고른 것(모르면 `?`). 다른 폴더를 고르면 재시작 확인 창 → 확인 = `/cmd/restart urdf:=<폴더>` (test-faker가 그 인자만 바꾸고 `map:=` 등은 그대로 · 빌드 후 띄움), 취소 = 원래대로. 새 `/robot_description`이 오면 카메라 렌더 몸·3D 뷰·수동 조작 뷰가 새 모델로 바뀜 |
| 상태 · 진단 | 작업 · 계획 · 운영 모드 · 위치 · 팔 · OpenCR(fake_opencr) / `/diagnostics_agg` 노드별 OK·WARN·ERROR |

주고받는 파일:

| 방향 | 파일 | ROS 쪽 |
| --- | --- | --- |
| 시뮬레이터 → ROS | `debug/input/hardware/camera/<카메라>_<0·1>.png` · `<카메라>_<0·1>_depth.png` (16비트 mm, 두 벌 번갈아) + 마지막에 메타 `<카메라>.json` {seq, stamp(촬영 시각), hfov_deg, color, depth} — fake_camera는 메타가 바뀔 때만 그 쌍을 한 번 발행, 메타 없으면 아무것도 안 냄 (지어낸 회색·0.5m 없음) | faker `fake_camera` → RealSense 토픽 (컬러 · 정렬 depth · 점구름 · camera_info) |
| 시뮬레이터 → ROS | `debug/input/topic/<토픽>/message` (`{"type", "data"}` JSON) | faker `input_manager` → `/cmd/*` (외부 명령은 전부 `/cmd/` 아래 — user_command · manual/cmd_vel · manual/arm · e_stop · operation_mode · map_mode · fake_opencr/set_pose) |
| ROS → 시뮬레이터 (읽기만) | `debug/output/hardware/serial`(C) · `debug/input/hardware/serial`(S) — 시리얼 장비 창 · `/fake_opencr/ground_truth`·`bumped` (진짜 위치) | 가상 시리얼 ↔ BeebotSystemHardware ↔ **fake_opencr** (faker 기본, OpenCR은 이것 하나) |
| 시뮬레이터 → ROS (요청) | 환경 → `debug/input/service/fake_opencr/set_parameters/message` · 위치 → `debug/input/topic/cmd/fake_opencr/set_pose` | fake_opencr 파라미터(slip·friction·comm_down·arm_grabbed·hw_error) · `/cmd/fake_opencr/set_pose` |
| ROS → 시뮬레이터 | `debug/output/topic/<토픽>/message` · `image.jpg` · `depth.png` + `debug/output/topic/list.json` | faker `monitor_manager` (모든 토픽의 마지막 메시지 + 목록) |
| ROS → 시뮬레이터 | `debug/service/list.json` · `service/<이름>/type|info` · `debug/action/list.json` · `action/<이름>/type|info` | faker `service_action_monitor` (서비스·액션 목록·정의) |
| 시뮬레이터 → ROS | `debug/input/service/<이름>/message` (`{"request_id", "data"}`) → `debug/output/service/<이름>/message` (`{"request_id", "ok", "data"\|"error"}`) · 기록 `debug/input/service/<이름>/history` (`>>request:<시각>>>` / `<<response:<시각><<` 다음 줄에 JSON, 계속 쌓임) | faker `service_action_input` → `/score_grasps` · `/clear_octomap` · `/apply_planning_scene` · `/reset` · `set_odometry` · `fake_opencr/set_parameters` · `map_saver/save_map`. ROS에 떠 있는 서비스·액션마다 `debug/input/service|action/<이름>/` 폴더를 2초마다 만들고, 타입을 등록 안 한 것에 요청하면 '지원 안 함' 오류로 답함 |
| 시뮬레이터 ↔ ROS | `debug/input/action/<이름>/goal` + `cancel` → `debug/output/action/<이름>/accept` · `feedback`(최신 1건) · `result` · 기록 `debug/input/action/<이름>/history` (`>>goal` · `>>cancel` · `<<accept` · `<<result`, feedback 제외) | faker `service_action_input` → `/navigate_to_pose` · `/move_action`(MoveIt) · `/execute_task_solution` |
| ROS → 시뮬레이터 | `debug/interface/list` + `debug/interface/<종류>/list` + `debug/interface/<종류>/<패키지>/<이름>/info` (예: `srv/ml/ScoreGrasps/info`, 프로젝트 타입은 자동 생성, 나머지는 `.../request` 쓰면 생성 — `bee.py interface <타입>`) | faker `service_action_monitor` (`ros2 interface list` 카탈로그) |

- 파일: `main.py`(메인 창) · `manual_window.py`(수동 조작 탭 패널) · `objects_window.py`(감지 물체 보기 창) · `perception_view.py`(인지 결과 읽기·3D 겹치기 공용) · `robot_view.py`(URDF 3D 뷰 · 바퀴 회전 표시) · `scene.py`(카메라 렌더) · `opencr.py`(fake_opencr 연결 — 읽기·요청만) · `maze.py`(코스) · `bridge.py`(debug 파일 주고받기)
- 팔·물체 상호작용 (`main.py` `grip_interact`, 매 렌더 스텝): 손끝 3cm 안에 + 높이도 맞는 퍽 + 그리퍼 닫힘 → 집어서 손끝 높이로 들어올림 (지도의 `held`, 카메라도 공중 퍽 렌더), 벌리면 그 자리에서 떨어짐 (트레이·데크·몸통 위면 안착, 아니면 바닥). 높이 안 맞으면 위를 지나가도 무시. 손끝이 물체 높이에 닿아 팔이 움직여야 밀려남 (퍽·바닥장애물, 벽은 안 뚫음; 베이스 이동분은 제외). 바퀴가 바닥 퍽에 닿으면 막지 않고 밀어냄 (벽에 끼면 막힘).
- 적재함 (`urdf` `tray_link` + `main.py` `TRAY`, 뒤쪽 절반): 내려놓으면 바닥면 (0.15m)에 안착, 가이드벽 4면 안쪽으로 가둠 (흘러넘침 방지), 로봇이 움직이면 함께 운반. 데크 (0.20m)·몸통 윗면 (0.145m)도 받침으로 동작. 지도에 외곽 표시.
- 물리는 컨테이너 `fake_opencr`: 바퀴는 명령 속도로 가속(한계 × 마찰), 엔코더는 바퀴 회전 그대로지만 바닥 이동은 (1 − 미끄럼)만큼 → `/odom`이 실제와 어긋난다 (rl_nav_correction·EKF가 다룰 문제). 시뮬레이터는 시리얼에 쓰지 않는다.
  로봇 원(반경 0.18m)이 벽·장애물·이동물체와 겹치면 그 걸음은 막힌다(충돌). 팔은 관절 속도 한계로 따라가고, 팔 붙잡기면 안 움직이고 과전류(→ arm_safety_monitor).
  통신 끊기면 C를 안 읽고 S를 안 보낸다 → 호스트 진단 "OpenCR 상태(S) 수신 없음" ERROR, 펌웨어 watchdog처럼 0.5초 뒤 바퀴 정지.
- faker를 다시 띄우면 `/odom`이 0부터 시작한다 — [로봇 위치 초기화]로 fake_opencr 위치·odom을 함께 맞춘다.
- 카메라 렌더: 로봇 URDF (`/robot_description`) 순기구학 + 관절값 + 로봇 위치 (fake_opencr가 돌면 진짜 위치 `/fake_opencr/ground_truth`, 아니면 `/odom`)로 두 카메라 자세를 구해 광선 추적 (`scene.py`, 2대 병렬로 약 0.3초 — 내부는 절반 해상도로 쏘고 320×240으로 확대).
  화면 타일은 SIM 렌더를 직접 보여준다 (ROS 왕복분 빠름, 타일에 SIM/ROS 표시). `input/hardware/camera` 파일 → `fake_camera` → ROS 토픽 경로는 그대로 돌아간다.
  URDF visual 상자로 **자기 몸 (팔·폴·상판)도 그린다** — 뒤쪽 상판에 실은 퍽 확인용. 렌더 중인 카메라 자체 마운트만 제외.
  자기 몸은 카메라 기준으로 관절값에만 달려서, 관절값 (바퀴 제외, 0.01rad 단위)이 같으면 몸 광선 추적 결과 (거리·색·법선)를 재사용하고 음영만 지금 자세로 다시 계산한다 (`Renderer.render(body_key=…)`). pole_cam이 몸을 내려다보면 메시 광선 추적이 프레임당 약 1초 (MPS 실측)라, 매번 하면 카메라가 약 2Hz로 떨어지고 collision_monitor가 점구름 끊김 (`invalid source`)으로 로봇을 세웠다. 재사용하면 약 40ms. 확인: `.venv/bin/python test_scene.py` (재사용한 렌더 = 안 한 렌더).
  화각 `HFOV_DEG`는 프레임 메타에 같이 써서 fake_camera가 그대로 쓴다 (따로 맞출 값 없음).
- 코스는 저장소의 `course/twin_closed_maze/walls.json`을 읽는다 (네트워크 불필요) — [course/README.md](../../course/README.md).
- map·odom 원점 = 미로 시작 칸으로 가정 (위치추정이 붙기 전까지). 퍽은 시작 칸이 있는 절반에만 놓는다 (가운데 벽이 위·아래를 막음).
- 실기 로봇에 붙여 보기: 로봇과 같은 네트워크의 PC에서 `ros2_ws/monitor.sh` (monitor_manager만) → 시뮬레이터가 상태·영상·진단을 표시

## 서비스·액션 입력 (파일로 호출·전송)

`bridge.py` 헬퍼가 `request_id`/`goal_id`(전문번호)를 자동 생성한다. 시작 전 잔재 파일은 무시, 서비스·액션마다 1건씩 순차 처리(처리 중 새 요청은 `busy` 에러).

```python
from bridge import send_service_request, read_service_response, send_action_goal, cancel_action_goal, read_action_file

qid = send_service_request('score_grasps', {'object_id': 3, 'max_candidates': 3})
resp, age = read_service_response('score_grasps')   # resp['request_id'] == qid 확인
# resp → {"request_id": ..., "ok": true, "data": {"scores": [0.87, ...], "grasps": [...]}}

gid = send_action_goal('navigate_to_pose', {'x': 1.0, 'y': 0.5})  # 간편형 x,y(+yaw) 또는 pose 전체
acc, _ = read_action_file('navigate_to_pose', 'accept')      # {"goal_id": ..., "accepted": True}
fb, _ = read_action_file('navigate_to_pose', 'feedback')      # 스트리밍 최신 1건 (faker는 안 냄, 실 Nav2만)
res, _ = read_action_file('navigate_to_pose', 'result')       # {"goal_id": ..., "status": "succeeded|aborted|canceled", ...}
cancel_action_goal('navigate_to_pose', gid)
```

- service `data`: `score_grasps`는 `object_id`(+`max_candidates`), `clear_octomap`은 `{}` (빈 요청), `apply_planning_scene`은 `{}` 또는 `{"is_diff": true}`
- action `data`: `navigate_to_pose`는 `{"x","y"[,"yaw"]}` 또는 `{"pose": {...}, "behavior_tree": ...}`, `execute_task_solution`은 `{"task_id": "..."}` (faker는 task_id만 보고 실행)
