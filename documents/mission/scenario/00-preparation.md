# 사전 준비 — 경기 전 지도 제작 (SLAM)

> 경기 당일 전에 **딱 한 번** 수행. 경기장 구조(벽)가 바뀌지 않는 한 재사용 가능.
> 퍽·신호등·목적지는 경기마다 랜덤 배치되므로 **지도에 포함하지 않는다.**

---

## 전체 흐름

```mermaid
flowchart TD
    START(["사전 준비 시작\n(경기장 세팅 완료 후)"])

    subgraph PRE1["① 경기장 준비"]
        WALL["벽·기둥 고정 완료\n(경기 구조물만 — 퍽·신호등 없이)"]
        ROBOT["로봇을 경기 시작 위치에 배치"]
    end

    subgraph PRE2["② SLAM 실행"]
        SLAM_START["map_mode = MAPPING 으로 bringup\n(RTAB-Map MAPPING 모드)"]
        MANUAL["수동 조종 (MANUAL 모드)\n경기장 전체 구역 천천히 주행"]
        COVER["모든 복도·코너 커버\npole_front_cam + gripper_cam\n(scan_pose로 후방도 촬영)"]
        CHECK["지도 품질 확인\n벽이 선명하게 그려졌는지"]
    end

    subgraph PRE3["③ 지도 저장"]
        SAVE["서비스 호출\n/map_saver/save_map\n→ rtabmap.db 저장"]
        FOLDER["저장 위치\n/ws/map/<지도이름>/rtabmap.db"]
        VERIFY["저장된 지도 확인\nRViz로 시각화"]
    end

    subgraph PRE4["④ 위치추정 검증"]
        LOC_TEST["map_mode = LOCALIZATION 으로 재시작\n(저장한 지도 로드)"]
        POSE_OK["로봇 위치가 지도에 맞게\n표시되는지 확인\n(RViz /initialpose 설정 필요 시)"]
        NAV_TEST["간단한 Nav2 이동 테스트\n목표 지점 몇 곳 이동 확인"]
    end

    DONE(["✅ 사전 준비 완료\n경기 당일: LOCALIZATION 모드로 시작"])

    START --> WALL --> ROBOT
    ROBOT --> SLAM_START --> MANUAL --> COVER --> CHECK
    CHECK -->|"불충분"| MANUAL
    CHECK -->|"충분"| SAVE --> FOLDER --> VERIFY
    VERIFY -->|"문제 있음"| SLAM_START
    VERIFY -->|"OK"| LOC_TEST --> POSE_OK --> NAV_TEST
    NAV_TEST -->|"이상 있음"| LOC_TEST
    NAV_TEST -->|"OK"| DONE
```

---

## SLAM 주행 요령

```mermaid
flowchart LR
    subgraph TIPS["주행 시 주의사항"]
        T1["🐢 천천히\n너무 빠르면 포인트클라우드 누락"]
        T2["↩️ 모든 코너 진입\n복도 끝까지 들어갔다 나오기"]
        T3["🔄 루프 클로저\n같은 구역을 여러 방향에서 촬영\n→ RTAB-Map 루프 감지 품질 향상"]
        T4["💡 조명 고정\n경기 당일과 동일한 조명 조건"]
        T5["🚫 퍽·신호등 없이\n이동하는 물체 없는 상태에서 제작"]
    end
```

---

## 경기 당일 준비

```mermaid
flowchart TD
    DAY(["경기 당일"])

    SETUP["퍽·신호등·목적지 배치\n(운영자가 랜덤 배치)"]
    BRINGUP["bringup 실행\nmap_mode = LOCALIZATION\nmap_name = <저장한 지도 이름>"]
    INIT_POSE["초기 위치 설정\n로봇을 정해진 시작 위치에 배치\n(RTAB-Map이 자동으로 매칭)"]
    TF_STABLE["map → odom TF 안정 확인"]
    ARM_P["팔 → initial_pose"]

    EXPLORE_GO["Phase 1 탐색 시작\n(요소 위치 파악)"]

    DAY --> SETUP --> BRINGUP --> INIT_POSE --> TF_STABLE --> ARM_P --> EXPLORE_GO
```

---

## 지도 관련 명령 참고

| 작업 | 방법 |
|------|------|
| SLAM 시작 | `map_mode = MAPPING` 으로 bringup |
| 지도 저장 | `/map_saver/save_map` 서비스 호출 |
| 지도 전환 | `/cmd/map_name <이름>` 토픽 발행 → state_manager가 RTAB-Map에 로드 |
| LOCALIZATION 전환 | `/cmd/map_mode LOCALIZATION` 토픽 발행 |
| 지도 목록 확인 | `/state/map_list` 토픽 구독 |
| 현재 지도 확인 | `/state/map_name` 토픽 구독 |
