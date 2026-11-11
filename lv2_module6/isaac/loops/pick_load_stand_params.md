# pick · load · stand_grasp · place 파라미터 정리

`stand_loop.py`·`pick_loop.py`·`place_loop.py`로 실기를 돌릴 때 건드리는 값을 한곳에 모았다. 2026-11-10 기준 `feature/isaac-setup` 작업 트리 값이다.
값의 출처는 아래 파일들이고, 여기서 바꾼 값이 아니라 **각 파일의 현재 값**을 옮긴 것이다.

| 구분 | 파일 |
| --- | --- |
| skill 노드 파라미터 | `ros2_ws/src/skills/config/skills.yaml` |
| 팔 공통값 (모든 팔 skill에 같이 들어감) | `ros2_ws/src/skills/params/arm.yaml` |
| 적재함 칸·carry·stow 자세 | `ros2_ws/src/skills/params/basket.yaml` |
| 학습 정책 켜고 끄기 | `models/skills/<정책>/meta.yaml` (`use: true`/`false`) |
| 그리퍼 컨트롤러 | `ros2_ws/src/ros_system/config/controllers.yaml` |
| 그리퍼 펌웨어 | `firmware/beebot_opencr*/config.h` |

노드 이름은 `skill_<이름>`이고 액션은 `/skills/<이름>`이다. 실행 중에 바꾸는 법은 `ros2 param set /skill_<이름> <파라미터> <값>`이다.
"매 goal마다 읽음"이라고 적은 값만 스택을 다시 띄우지 않고 바뀐다. 나머지는 노드가 시작할 때 한 번만 읽는다.

## 1. 책상 시험 임시 값 (11-10 로봇 복귀로 되돌림)

팔을 몸체에서 떼어 책상에서 시험할 때 바꿨던 값은 로봇에 다시 달면서 모두 되돌렸다: `stand_box_top_m` 0.07, `standing_grasp_z` 받침 0.07, perception `location_box_size`(기본값), `controllers.yaml` `sag_enabled` true.
책상과 상관없어 남긴 값은 아래와 같다.

| 항목 | 현재 값 | 이유 |
| --- | --- | --- |
| `stand_flip_roll` | true | 쥐는 동안 손 카메라가 퍽 몸통을 보게 (11-10 실기 확인). `stand_grasp` v1 정책도 이 방향으로 학습 |
| `stand_grasp` 정책 | 켜짐 (`use: true`) | 11-10 도달 학습 v1 |

## 2. 팔 공통 (pick · load · stand 모두)

`arm.yaml`과 `ArmIO`(`src/arm_io.cpp`)가 선언한다.

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `arm_velocity_scale` | 1.0 (`arm.yaml`) · 코드 기본 0.3 | MoveIt 팔 계획 속도·가속도 비율 (0~1]. 계획할 때마다 읽음 |
| `arm_plan_tries` | 3 | 계획이 실패하면 같은 목표로 다시 시도하는 횟수 |
| `contact_mode` | `sim` | `sim` = 퍽·칸에 닿는 구간은 시뮬과 같은 기준 동작 + 학습 보정, `moveit` = MoveIt만 |
| `grip_current_a` | 0.1 A | 닫을 때 쥐는 힘 상한 (약 7 N) |
| `grip_close_gap_m` | 0.026 m | 닫는 목표 손가락 사이. 퍽 30 mm보다 4 mm 좁게 두고 힘은 전류로 제한 |
| `grasp_min_pos_m` | 0.003 m | 이보다 더 닫히면 빈손 |
| `grasp_min_effort` | 0.25 A (stand_test 기본) · Isaac 0 | 잡기 판정 전류 하한. 0이면 위치·멈춤만으로 판정하고 `stand_grip_current_a`도 0이 됨 |
| `grasp_check` | true | 잡기 판정 사용 여부 (launch 인자, faker는 false) |
| `grasp_stop_margin_m` | 0.0005 m | 닫는 목표보다 이만큼 넘게 못 들어가고 멈추면 쥠으로 판정 |
| `server_wait_s` | 30.0 s | MoveIt·Nav2 서버가 뜰 때까지 기다리는 시간 |
| `pose_look` | `[0, -1.6825, 0.3246, 1.9268, 0]` | inspect가 처음 보는 자세 |
| 상수 `GRIPPER_OPEN_M` / `GRIPPER_CLOSED_M` | 0.0265 / 0.0 m | URDF 한계 (손가락 하나) |
| 상수 `TRAJ_DT` | 0.02 s | 궤적 샘플 간격 (정책 주기 50 Hz) |
| 상수 `CONTACT_ABOVE_M` | 0.05 m | 닿는 구간 시작 높이 (쥐는 손끝 위) |

## 3. pick (`skill_pick`, 서 있는 퍽 집기)

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `max_grasp_tries` | 2 | 파지 후보를 점수순으로 몇 개까지 시도 |
| `pick_via_carry` | true | 적재함 쪽 팔이면 carry 자세를 멈추지 않고 지나 퍽 위로 감 |
| `pick_open_gap_m` | 0.060 m (`arm.yaml`) · 코드 기본 0.040 | 닿는 구간 전 손가락 사이. 인식 오차 ~10 mm에서 손가락이 퍽 윗면에 걸리지 않게 넓힘. 최대 약 0.070. 실행 중 변경 가능 |
| `pick_above_m` | 0.05 m (코드 기본 `CONTACT_ABOVE_M`) | 닿는 구간 시작 높이. 0.04면 도달 반경이 약 8 mm 늘어남 (x 256 → 264 mm). 실행 중 변경 가능 |
| `pose_carry_left` / `pose_carry_right` | `basket.yaml` | 적재함 쪽 경유 자세 |
| 정책 `pick_standing_puck` | **켜짐** | 팀원 `pick_residual_sag`. 시뮬 기준 99.1% → 정책 100.0% (1024판) |

실패 사유: `no_grasp`, `pick_unreachable`(손을 아래로 향한 채 퍽 위 자세 IK 실패 — 후보 전부. 이번 커밋 안 된 수정), `arm_contact`(과전류 접촉), `grasp_empty`(빈손), `grasp_failed_all`.

## 4. load (`skill_load`, 적재함에 넣기)

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `server_wait_s` | 30.0 s | 서버 대기 |
| `load_lift_m` | 0.03 m | 박스 앞에서 시작하면 손을 곧게 이만큼 올린 뒤 carry로 감. pick 끝의 손 안 퍽 바닥이 박스 위 5 cm이므로 +3 cm = 8 cm. 0이면 안 올림 |
| `load_end_pose` | `stow` | 넣은 뒤 끝 자세. `stow` · `carry`(칸 쪽 옆 자세) · `over`(칸 위 그대로). 실행 중 변경 가능 |
| `load_via_carry` | true | 집은 자리에서 carry를 거쳐 칸 위로. false면 칸 위(`q_over`)로 바로 (손 안 퍽은 MoveIt이 모름 — 부딪힘 주의) |
| `carry_blend` | true | carry를 멈추지 않고 지나는 관절 공간 베지어. false면 carry에서 멈췄다 감 |
| 상수 `LOAD_OPEN_M` | 0.015 m | 놓을 때 손가락 간격 50 mm. 펌웨어 그리퍼 표의 38~75 mm 구간은 실측 없는 보간이라 몇 mm 어긋날 수 있음 |
| 정책 `load_slot` | **켜짐** | 시뮬 94.8% → 99.0%, 되돌릴 수 없는 실패 0.49% |

적재함 자세는 `basket.yaml`에 있다.
- `pose_stow = [0, -1.9024, 1.344, 0.9023, 0]`
- `pose_carry_left = [1.5708, -0.6981, 0.3491, 1.0472, 1.5708]`, `pose_carry_right`는 부호 반대
- `basket_carry` (칸별 left/right), `basket_box_q_over`, `basket_box_q_low` (칸 0~11 × joint1..5)

load 순서: carry → `q_over` → `q_low`(퍽 바닥이 칸 바닥 위 15 mm) → 벌림 → `q_lift` → stow. `slot_table.py`로 표를 다시 만들면 `basket.yaml`도 같이 바꿔야 한다.

## 5. stand (`skill_stand`, 눕힌 퍽 세우기) 와 stand_grasp 정책

### 5-1. 정책 `stand_grasp`

| 항목 | 값 |
| --- | --- |
| 역할 | 누운 퍽 먼 끝 모서리를 쥘 때 위에서 멈추고 내려가는 구간의 팔 보정 (원호는 기준 동작) |
| 현재 `use` | **false** (끔) |
| 시뮬 성능 (1024판, 계획 위치 ±8 mm · yaw ±6°) | 정책 없이 61.6% → 정책 83.4% (옆 오차 p90 8.1 → 3.9 mm) |
| 켜면 달라지는 것 | 내려가기가 contact 속도 0.3으로 느려짐 (`stand_vel_scale` 0.7 대신) |
| 입력 | pick 모양(41), 카메라 = 쥘 끝 − 손끝 (`hand_relative` `box_tag`) |
| 상태 | 실기 확인 전. 책상 높이로는 아직 검증하지 못함 |

같은 노드의 다른 정책 `regrasp_after_stand`(입력 모양이 없고 합격 기준 미달)는 `use: false`다.

### 5-2. 위치·높이

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `stand_waypoints` | `ik` | `ik` = 해석 IK, `table` = `params/stand_table.yaml` |
| `stand_box_top_m` | 0.085 (임시) | 퍽이 놓인 면 높이. 노드 시작 때만 읽음 |
| `stand_puck_dx_m` | −0.010 m | inspect 퍽 x 위치에 더하는 보정 (inspect가 손 카메라보다 10~13 mm 앞) |
| `stand_contact_h_box_m` / `_cyl_m` | 0.030 / 0.030 m | 손가락 중심 높이 (퍽 바닥 기준). 사각은 윗면, 원기둥은 축 |
| `stand_above_m` | 0.04 m | 대기 높이 (못 닿으면 3→2→1 cm로 낮춤) |
| `stand_lift_away_m` | 0.08 m | 손 빼기 높이 (못 닿으면 6→4→3 cm) |
| `stand_end_margin_m` | 0.03 m | 선 퍽 가장자리가 상자 가장자리에 이보다 가까우면 반대 끝을 잡음. 0이면 끔 |

### 5-3. 파지 (그리퍼)

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `stand_open_m` | 0.012 m | 벌림 (손가락 하나, 간격 44 mm) |
| `stand_pinch_m` | 0.0062 m | 쥐는 목표 (간격 30 mm). 안 닿아 빠지면 0.0065, 세면 0.0075 |
| `stand_close_current_a` | 0.01614 A | 목표까지 닫을 때 전류. 10.76 mA는 안 닫힌 라운드가 있었고 5.38 mA는 안 움직임 |
| `stand_pinch_current_a` | 0.00538 A | 쥠을 확인한 뒤 낮춰 쥐는 전류 (약 0.39 N) |
| `stand_grip_current_a` | 0.007 A | 이 전류 이상 + 0.2 s 안 움직이면 쥠. 퍽을 밀 때는 8~10 mA |
| `stand_empty_m` | 0.003 m | 이보다 닫히면 빈손 → `stand_no_grip` |
| `stand_grip_timeout_s` | 4.0 s | 넘으면 `stand_grip_timeout` |
| `stand_hold_s` | 0.3 s | 퍽 위에서 멈춰 처짐 보정을 쌓는 시간 |

### 5-4. 호 (세우는 동작)

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `stand_arc_end_deg` | 100° | 호 끝 각도 |
| `stand_arc_k` | 1.0 | 호의 퍽 축 방향 배율. 1 원, 0.5 반, 0 쥔 점 그대로 위로 |
| `stand_tilt_over_m` | 0.030 m | 호를 돌며 끝까지 더 올림 (시뮬 0.010) |
| `stand_arc_return` | false | 호 끝에서 90°로 되돌리지 않고 바로 벌림 |
| `stand_set_dz_m` | −0.020 m | 끝점에서 눌러 놓음 |
| `stand_drag_m` | 0.0 m | 0이면 호. 양수면 잡은 채 로봇 쪽으로 곧게 끌기 |
| `stand_drag_lift_m` | 0.035 m | 끌면서 올리는 양 (drag > 0일 때) |
| `stand_vel_scale` | 0.7 | 내려가기·위로 속도 × 팀 관절 한계 |
| `stand_arc_vel_scale` | 0.7 | 끌기·호 속도 (시뮬: 0.3 → 63%, 0.5 → 100%) |

### 5-5. 손 카메라·쥘 끝 선택

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `stand_flip_roll` | true (임시) | 손목 roll 180° — 카메라가 퍽 몸통 쪽에 옴. 닫는 축은 그대로. 매 goal마다 읽음 |
| `stand_cam` | false | 내려가기 전 손 카메라로 퍽 위치를 다시 잼 |
| `stand_cam_above_m` / `_back_m` | 0.06 / 0.025 m | 잴 때 손끝 높이 / 로봇 쪽으로 물러나는 거리 |
| `stand_cam_wait_s` / `_min_n` | 1.2 s / 2 | 재는 시간 / 이만큼 못 보면 inspect 값 사용 |
| `stand_cam_max_m` / `_max_deg` | 0.03 m / 30° | 이보다 많이는 위치·축을 옮기지 않음 |
| `stand_score_margin` | 3.0 | 쥘 끝 점수 가중치: 선 자리 여유 |
| `stand_score_slack` | 1.0 | 쥘 끝 점수 가중치: 관절 한계 여유 |
| `stand_score_cam` | 2.0 | 쥘 끝 점수 가중치: `stand_flip_roll` 방향 |
| `stand_score_far` | 0.2 | 쥘 끝 점수 가중치: 먼 끝 (동점이면 기존대로) |


실행 중에 바꿀 수 있는 값 (매 goal마다 읽음): `stand_open_m`, `stand_pinch_m`, 세 전류, 호·끌기 관련, `stand_flip_roll`, `stand_vel_scale`, `stand_arc_vel_scale`, `stand_hold_s`. 읽는 목록은 `stand_skill.cpp`의 `refresh_params()`다.

## 6. place (`skill_place`, 손에 든 퍽 내려놓기)

`place_loop.py`가 쓴다. goal의 위치는 놓을 면이다(z = 면 높이, 손 안 퍽 바닥이 여기에 닿는다). 아래 값은 모두 노드가 시작할 때 한 번만 읽으므로, 바꾸면 스택을 다시 띄워야 한다.

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `held_bottom_m` | 0.0606 m (`skills.yaml`) | 손끝에서 손 안 퍽 바닥까지 |
| `place_release_m` | 0.02 m (`skills.yaml`) | 벌리는 높이. 퍽 바닥이 면 위 이만큼일 때 벌림 (`contact_mode moveit`, target `over`) |
| `place_approach_m` | 0.08 m (`skills.yaml`) | `contact_mode moveit`일 때 위에서 내려오기 시작하는 높이 |
| `place_above_m` | 0.08 m (코드 기본) | `contact_mode sim`일 때 손 안 퍽 바닥이 면 위 이만큼에서 다가와 수직으로 내려감. 이웃 퍽(60 mm)보다 높아야 옆으로 지나갈 때 안 침 (11/08 실기 주사위 5: 5 cm로 먼 줄 퍽을 넘어뜨림). 손이 못 닿으면 5 cm(`CONTACT_ABOVE_M`) |
| `place_offset_x_m` | 0.0037 m (코드 기본) | 목표를 로봇 쪽으로 이만큼 당김. 시뮬에서 퍽이 칸 가운데보다 앞으로 놓이던 만큼이며, 실기에서 자로 재서 다시 정할 값 |
| `place_via_carry` | true | 팔이 적재함 쪽(\|joint1\| > 1.2 rad, `unload_end_pose over`)이면 carry를 멈추지 않고 지나 놓을 자리 위로 감 |
| `pose_carry_left` / `pose_carry_right` | `basket.yaml` | 위의 경유 자세 |
| `contact_mode` | `sim` (`arm.yaml`, 팔 공통) | `sim`: 칸 위까지 MoveIt, 그다음 내려가기 → 벌림 → 올리기를 기준 동작(+정책)으로. `moveit`: MoveIt만 (예전 방식) |
| 상수 `PLACE_OPEN_GAP_M` | 0.036 m | 놓을 때 손가락 간격. 이웃 퍽 사이 틈(30 mm)에 손가락이 들어가는 폭 |
| 정책 `place_cell` | 꺼짐 (`use: false`) | 팀원 `place_residual_sag`. 시뮬 기준 96.3% → 정책 99.0%. 상수 오프셋만으로 Δ = 0 99.7%라 정책 없이 감 |

손가락은 늘 로봇 y 방향으로 닫는다(시뮬 `place_env`와 같음). target은 `cell`(주사위 칸, 좌표 지정)과 `over`(물체 위: location_box·판, `values[0]`만큼 위에서 다가감) 두 가지다.
`place_cell`은 처짐 적분(sag)을 0으로 되돌린 뒤 시작하는데, 정책이 꺼져 있으면 되돌리지 않는다(되돌리면 팔이 처졌다 올라오며 약 0.45 s 멈춤).

실패 사유: `place_bad_target`, `arm_unavailable`, `place_tf`(로봇 방향 TF 없음), `arm_contact`(과전류 접촉), `place_failed`.

`--from slot`이면 앞에 `skill_unload`가 돈다.

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `unload_end_pose` | `carry` (코드 기본) | 꺼낸 뒤 끝 자세. 실행 중 변경 가능. `place_loop.py --auto`는 칸 위(`over`)에서 끝내고 place가 carry를 지나게 함 |
| `server_wait_s` | 30.0 s | 서버 대기 |
| 정책 `unload_slot` | 꺼짐 (`use: false`) | 시뮬 32.5% → 97.0%. 11-08 첫 실기 시험은 정책 없이 |

`place_loop.py` 실행 인자 (스택은 먼저 `./manipulator-launcher.sh`로 띄움):

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--from slot\|pick` | (필수) | 퍽을 가져올 곳. `slot` = 적재함 칸에서 unload, `pick` = stow → inspect → 상자 위 퍽 집기 |
| `--to X Y Z` | 0.22 -0.05 0.07 | 놓을 자리 (`base_footprint`, 퍽 중심 x·y, 면 높이 z) [m] |
| `--slot` | 0 | `--from slot`: 첫 칸 (0~11). 라운드마다 +1 |
| `--shape` | box | `--from slot`: 칸마다 퍽 모양 (`box box box cyl cyl`). 하나만 주면 전부 |
| `--box-x` / `--box-y` | 0.22 / 0.0 | `--from pick`: 상자 중심 [m] |
| `--support` | 0.07 | `--from pick`: 상자 윗면 높이 [m] |
| `--puck-h` | 0.06 | 서 있는 퍽 높이 [m] |
| `--dice` | 0 (끔) | 1~6: 주사위 눈 모양으로 놓음. k번째 라운드가 k번째 눈, 먼 줄부터 (`--to`가 무늬 가운데) |
| `--pitch` | 0.045 | `--dice` 눈 간격 [m]. 실제 판의 0.060은 주행 없이 안 닿음 (먼 줄 x > 0.25) |
| `--rounds` | 0 | 0이면 q나 Ctrl-C까지 |
| `--auto` | 끔 | 라운드 사이 Enter 없음. unload → place를 이어서, 첫 실패에서 멈추고, 자 측정값은 끝에 한 번 |
| `--bag` | motion | 라운드마다 팔 동작 bag (`--auto`면 한 개). `off` = 안 함 |
| `--use_sim` | false | Isaac Sim beebot (sim time) |

예:

```bash
python3 place_loop.py --from pick --to 0.22 0.05 0.07
python3 place_loop.py --from slot --slot 0 --shape box box box cyl cyl --to 0.22 -0.05 0.07
python3 place_loop.py --from slot --shape box box box cyl cyl --dice 5 --to 0.185 0 0.07 --auto
```

## 7. 그리퍼 하드웨어 쪽 값

| 항목 | 값 | 위치 |
| --- | --- | --- |
| `gripper_controller` `max_effort` | 0.6 A | `controllers.yaml` — 액션 goal에 effort가 없을 때의 쥐는 힘 한계 |
| `allow_stalling` | true | 퍽을 쥐어 손가락이 멈추는 것을 실패가 아닌 파지로 봄 |
| 펌웨어 기본 쥐는 전류 | 600 mA | `config.h` `GRIPPER_GOAL_CURRENT_MA` |
| 펌웨어 전류 범위 | 2.69 ~ 1200 mA | `GRIPPER_CURRENT_MIN_MA` / `MAX_MA` (C 줄 9번째 칸). 수십 mA 아래는 마찰로 안 움직일 수 있음 |
| 손가락 ↔ 서보각 표 | 실측 5점 | `GRIPPER_FINGER_M`, `GRIPPER_SERVO_RAD`. 75 mm는 실측 안 한 보간 |

## 8. 값을 바꿀 때

1. 실행 중에 바꾸려면 `ros2 param set /skill_stand stand_pinch_current_a 0.0075`처럼 한다 (노드 이름은 `skill_stand`, `skill_pick`, `skill_load`, `skill_place`, `skill_unload`).
2. 영구히 바꾸려면 위 표의 yaml을 고치고 스택을 다시 띄운다. `colcon build --symlink-install`로 설치했으면 yaml은 링크라 재빌드가 필요 없다.
3. 정책 `use`와 헤더(`stand_plan.hpp`)는 `colcon build --symlink-install --packages-select skills`가 필요하다.
4. `stand_loop.py`는 `ros2 param set`으로 바꾼 `stand_*` 값을 매 라운드 CSV에 같이 기록한다 (`PARAMS` 목록).
