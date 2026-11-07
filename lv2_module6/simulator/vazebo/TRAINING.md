# YOLO 학습 (host) — export → train → perception 장착

호스트(Mac, MPS)에서 돌린다. 컨테이너(Docker)는 학습 안 함 — 가중치 파일만 받아간다.

## 0. 준비 (최초 1회)

```bash
cd lv2_module6/simulator
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

## 1. 데이터셋 생성 (`export_dataset.py`)

test-faker 기동 상태에서 실행 — 시뮬 렌더를 `pole_cam`에 투영해 자동 라벨 생성.

```bash
# 컨테이너 (저장소 루트에서)
cd docker && docker compose up -d && docker compose exec lyrical bash
test-faker 0   # 끄지 말 것 — 데이터셋 뽑는 동안 유지
# 호스트 (새 터미널)
cd lv2_module6/simulator
.venv/bin/python export_dataset.py
```

산출물: `dataset/{train,val}/{images,labels}/` + `dataset/data.yaml` (nc=5:
`puck, goal_zone, obstacle, person, traffic_light`). 기본 약 1174장(train 1057/val 117).

## 2. 학습 (`train_yolo.py`)

```bash
cd lv2_module6/simulator
.venv/bin/python train_yolo.py [--epochs 100] [--model yolo11n.pt] [--imgsz 640] [--device mps]
```

- 출력 고정: `simulator/runs/<name>/weights/best.pt`
  (`train_yolo.py:34`에서 `project=HERE/runs` 절대경로 고정 — cwd가 어디든 여기로 간다)
- 끝나면 스크립트가 자동으로 `../../ros2_ws/model/yolo/model.pt`에 복사한다 (`train_yolo.py:35-39`)
- MPS(Mac GPU) 기준 100에폭 약 1.5시간. 로그는 nohup으로: `nohup ./.venv/bin/python train_yolo.py --epochs 100 > /tmp/train.log 2>&1 &`

### 학습 현황 보기

```bash
tail -1 /tmp/train.log | tr '\r' '\n' | tail -2   # 에폭 진행률 (75/100 형태)
tail -1 runs/yolo11n-beebot/results.csv           # 지표 마지막 줄 (epoch·box_loss·mAP50·mAP50-95)
ls -la runs/yolo11n-beebot/weights/               # last.pt/best.pt 갱신시각
open runs/yolo11n-beebot/results.png              # 학습 곡선 그림
```

## 3. 컨테이너 perception에 장착

가중치는 빌드 때 `share/perception/model/model.pt`로 설치된다
(`src/perception/CMakeLists.txt:28-33`, `file(GLOB)` 방식). 그래서 **복사만으로는 안 되고 재빌드가 필수**다.
`GLOB`은 configure 때 확정되므로, 가중치가 새로 생겼으면 force-configure:

```bash
# 컨테이너 안
source /opt/ros/lyrical/setup.bash
cd /ws && colcon build --symlink-install --cmake-force-configure --packages-select perception
```

그 뒤 perception 노드 재시작 (launch가 respawn 안 함 — 죽어 있으면 직접):

```bash
ros2 run perception perception_node --ros-args \
  --params-file /ws/install/perception/share/perception/config/perception.yaml
```

`model_path` 파라미터가 비어 있으면 share의 `model/model.pt`를 자동 탐색한다
(`scripts/perception_node.py:58-66`, `config/perception.yaml:5`).
모델 없이 뜨면 "빈 목록 발행" 모드 — `/tracked_objects`가 `objects: []`로만 나온다.

## 4. 확인

```bash
# 컨테이너 안 — 15초 안에 class_name이 보여야 정상
timeout 15 ros2 topic echo --once /tracked_objects perception/msg/TrackedObjectArray | grep class_name
```

`puck`+`goal_zone`이 둘 다 있어야 `vlm_task_planner`가 계획 보류(`...대기 — 계획 보류`)에서 풀린다
(`src/plan/src/vlm_task_planner.cpp:375-382`).
시뮬레이터 지도에는 보라 테두리 + `클래스#id`로 표시된다 (`main.py:633-640`).

## 5. 삽질 기록 (재발 방지)

- **옛날 산출물이 `.../source/llm/runs/detect/`에 있음**: `project` 고정 전 스크립트를
  cwd=`.../source/llm`에서 돌려서 ultralytics 기본값(`cwd/runs`)으로 빠진 것.
  로그 증거: `args.yaml:14-16` (`project: null`), `save_dir=.../llm/runs/...`.
  현 스크립트는 절대경로 고정이라 재발 안 함. llm 쪽 `best.pt`는 같은 데이터셋 산출물이므로
  임시 가중치로만 쓰고, 정식 학습 끝나면 `simulator/runs` 산출물로 교체할 것.
- **빌드했는데 모델이 share에 없음**: `file(GLOB)` 탓. `--cmake-force-configure` 없이 빌드하면
  빈 모델 목록이 캐시된다. 증상: `ls /ws/install/perception/share/perception/model/` 비어 있음.
- **perception이 뜨자마자 죽음**: `AttributeError: ... 'warn'` — `get_logger().warn`은 없음.
  `warning`을 쓸 것 (`scripts/perception_node.py:79` 수정됨).
- **`/tracked_objects`가 계속 빔**: `to_map`이 `map` 프레임을 찾지 못한 것일 수 있다
  (`scripts/perception_node.py:209-222`). EKF가 `map→odom`을 안 주면 테스트에선
  `static_transform_publisher`로 메운다:
  `ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom`
  (positional 인자 형식 아님 — `--frame-id`/`--child-frame-id` 플래그).
- **VLM이 응답을 받고도 `/sub_goals`를 안 냄**: `try_vlm_async`의 epoch 갱신 위치 버그였음
  (수정됨, `vlm_task_planner.cpp:406-409` — busy 검사를 epoch 증가보다 먼저).
  증상: ai-service `log/plan-*/response.json`에는 정상 goals가 있는데 plan_id가 안 오름.
