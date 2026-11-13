# Foxglove 레이아웃

bringup이 띄우는 `foxglove_bridge`(기본 `ws://<로봇 IP 또는 bee.local>:8765`)에 붙어 미션을 보는 화면. RViz 대신 맥·아이패드에서 봅니다.

```text
foxglove/
├── bee-mission-layout.json        # 레이아웃 (탭 5개) — Foxglove에서 가져오기
└── userscripts/tracked_markers.ts # /tracked_objects → 3D 상자·글자 (레이아웃 안에 같은 코드가 들어 있음, 리뷰용 원본)
```

## 가져오기

1. Foxglove 열기 — 맥: 데스크톱 앱 또는 Chrome에서 app.foxglove.dev · 아이패드: Safari/Chrome에서 app.foxglove.dev (계정 로그인 필요)
2. **Open connection → Foxglove WebSocket** → `ws://bee.local:8765`
   - 아이패드에서 `ws://`가 막히면(https 페이지라서) 로봇을 `foxglove_tls:=true`로 띄우고 `wss://bee.local:8765` — 자체 서명 인증서라 먼저 Safari로 `https://bee.local:8765`를 한 번 열어 신뢰
3. 왼쪽 위 **Layouts → Import from file…** → `bee-mission-layout.json`
   - 아이패드: 파일을 AirDrop·iCloud Drive로 옮겨 두고 고르기. 한 번 가져오면 계정에 저장돼 맥·아이패드 같이 씀

## 탭

| 탭 | 보는 것 | 토픽 |
| --- | --- | --- |
| 미션 | 단계 · 신호 확정/오류 표시등, 미션 상태 전체, 적재함 12칸 표, 앞 카메라 검출 영상, 단계 흐름(시간축), 시작·정지 버튼 | `/mission/state` · `/mission/signal_status` · `/coordinator/basket` · `/perception/pole_front_cam/debug_image` |
| 주행 | 3D(지도 · 코스트맵 · 경로 · 미션 마커 · 로봇 모델 · 검출 물체), 속도 명령 vs 실제, 충돌 감시 | `/map` · `/global_costmap/costmap` · `/plan` · `/mission/markers` · `/robot_description` · `/cmd_vel` · `/odometry/filtered` |
| 인지 | 두 카메라 검출 영상, 융합 물체 표, 신호 사이클 결과, 주사위 사진 | `/perception/*/debug_image` · `/tracked_objects` · `/mission/dice_image` |
| 진단 | 진단 요약·상세, WARN 이상 로그, 배터리, 모터 오류·온도 | `/diagnostics_agg` · `/rosout` · `/motor_status` |
| 조작 | 비상정지 · 모드 · 단계 표시등, 미션 시작/정지, 비상정지 걸기/풀기, AUTO/MANUAL, 적재함 장부 비우기, 시작 거부 이유 | `/cmd/mission` · `/cmd/e_stop` · `/cmd/operation_mode` · `/coordinator/basket_clear` |

## 부하

- 브리지는 **화면에 떠 있는 패널이 구독한 토픽만** 보냅니다. 영상(`debug_image` 640×360 rgb8 ≈ 0.7 MB/장)은 Wi-Fi에서 가장 무거움 — 보지 않을 땐 다른 탭으로.
- `debug_image`는 perception이 **구독자가 있을 때만** 그림 (`perception_node.cpp` `get_subscription_count`). 원본 `color/image_raw`는 레이아웃에 넣지 않았고 압축 토픽(image_transport)도 지금 없음.
- 3D의 점군(`points_filtered`)은 꺼 둠 — 필요할 때만 켜기.

## 주의

- 버튼(Publish)은 진짜 로봇 토픽에 냅니다. `/cmd/mission`은 `start` · `abort`만 받음 (그 밖의 문자열은 무시).
- 레이아웃을 앱에서 고쳤으면 **Export…** 로 이 파일을 덮어써 커밋.
- 3D 검출 물체(`/foxglove_script/tracked_markers`)는 User Script — 패널 왼쪽 메뉴 **User Scripts**에서 오류를 볼 수 있음.
