# Phase 2 — 신호 읽기

> 목적: 신호등 깜빡임 3단계를 해석해 "어떤 색 퍽 몇 개를, 번호 합 얼마로, 어느 목적지로" 를 확정.

## 흐름

```mermaid
flowchart TD
    TL_FOUND["🚦 /tracked_objects\ntraffic_light 등록됨"]

    subgraph SD["signal_decoder"]
        WATCH["깜빡임 감시\n(색 상태: red·green·yellow·all·off)"]

        subgraph STAGE1["1단계 — 퍽 색과 개수"]
            S1["특정 색 N번 깜빡임"]
            S1_OUT["color = RED\ncount = N"]
        end

        subgraph STAGE2["2단계 — 번호 합"]
            S2["세 색 순서대로 깜빡임\n각 색 점수(기본 모두 1) 합산"]
            S2_OUT["sum = S"]
        end

        subgraph STAGE3["3단계 — 목적지"]
            S3["노랑 N번 깜빡임"]
            S3_OUT["dest_tag = 11+N\n(1→12, 2→13, 3→14)"]
        end

        SEP["단계 구분 신호\n세 색 동시(all) 2초\n또는 꺼짐 3초"]
    end

    RESULT["/mission/signal 발행\n{ color, count, sum, dest_tag, raw }"]

    TL_FOUND --> WATCH
    WATCH --> SEP
    SEP --> S1 --> S1_OUT --> SEP
    SEP --> S2 --> S2_OUT --> SEP
    SEP --> S3 --> S3_OUT
    S1_OUT & S2_OUT & S3_OUT --> RESULT
```

## 깜빡임 예시

```
신호등 시퀀스:
  [all 켜짐 2초]  ← 1단계 시작 신호
  빨강 ● 꺼짐 ● 꺼짐   → RED 2번
  [all 켜짐 2초]  ← 2단계 시작 신호
  빨강 ● 노랑 ● 초록 ● 초록 ● 빨강 ●  → 점수 합 5 (모두 1점)
  [all 켜짐 2초]  ← 3단계 시작 신호
  노랑 ● 노랑 ●   → 목적지 tag=13

결과: /mission/signal
  color    = "red"
  count    = 2
  sum      = 5
  dest_tag = 13
  raw      = "R R | R Y G G R | Y Y"
```

## 잡음 처리

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| `min_on_s` | 0.15s | 이보다 짧은 켜짐은 잡음으로 무시 |
| `separator_s` | 2.0s | all 신호가 이 이상 이어지면 단계 구분 |
| `gap_s` | 3.0s | 꺼짐이 이 이상이면 단계 끝 |
| `score_red` | 1 | 빨강 점수 (공지 후 변경 가능) |

## 신호를 못 읽으면?

```mermaid
flowchart LR
    MISS["깜빡임 놓침\n또는 노이즈"]
    RAW["raw 문자열 그대로\n/mission/signal 발행"]
    VLM["vlm_task_planner\n판단 불가 시 say\n→ 사람 확인 요청"]

    MISS --> RAW --> VLM
```
