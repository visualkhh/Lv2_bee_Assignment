# 플랜 시나리오 — 신호등 암호 미션 (v2)

> **전제 (v2)**
> - 지도(map)는 경기 전 미리 SLAM 완료 → 경기 시작 시 **LOCALIZATION** 모드
> - 퍽·신호등·목적지 위치는 경기마다 **랜덤 배치** → 탐색으로 발견
> - 신호등은 **심판/운영자가 버튼을 눌러야 점멸 시작**

---

## 문서 목록

| 파일 | 내용 |
|------|------|
| [00-preparation.md](00-preparation.md) | **사전 준비 — SLAM 지도 제작 (경기 전)** |
| [01-overview.md](01-overview.md) | 전체 흐름 다이어그램 (v1→v2 비교 포함) |
| [02-phase0-boot-and-explore.md](02-phase0-boot-and-explore.md) | Phase 0 부팅 + Phase 1 요소 탐색 |
| [03-phase2-signal.md](03-phase2-signal.md) | Phase 2 신호 읽기 (버튼 트리거) |
| [04-phase3-plan.md](04-phase3-plan.md) | Phase 3 vlm_task_planner 판단 (4가지 케이스) |
| [05-phase4-execute.md](05-phase4-execute.md) | Phase 4 coordinator 실행 |
| [06-verify-and-emergency.md](06-verify-and-emergency.md) | Phase 5 수납 확인 + 비상 시나리오 |
| [07-timeline.md](07-timeline.md) | 전체 타임라인 gantt |
| [08-nodes-per-phase.md](08-nodes-per-phase.md) | **단계별 노드 역할 명세 (입력·출력·담당자)** |
| [PLAN_SCENARIO.md](PLAN_SCENARIO.md) | 텍스트 통합본 (v1) |

---

## 한눈에 보는 흐름

```
부팅
 └→ RTAB-Map LOCALIZATION (미리 만든 지도 로드)
 └→ 팔 initial_pose

탐색 (EXPLORE)
 └→ explorer가 지도 안 미탐색 구역 순회
 └→ perception이 퍽·신호등·목적지 위치 발견
 └→ 신호등 발견 → signal_decoder 대기 진입

버튼 누름 (심판)
 └→ 신호등 점멸 시작
 └→ signal_decoder 3단계 해석
 └→ /mission/signal { color, count, sum, dest_tag }

계획 수립
 └→ 케이스 A: 조합 즉시 carry
 └→ 케이스 B: 태그 모름 → inspect 먼저
 └→ 케이스 C: 목적지 미발견 → wait
 └→ 케이스 D: 조합 불가 → say

실행
 └→ navigate → pick → place tray (×N) → navigate 목적지 → pick tray → place (×N)

수납 확인
 └→ 목적지 근처 퍽 번호 확인 (60초 주기)
```
