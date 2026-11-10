# 부록: 컴포넌트·액션·프로세스 정리 (skills 기준)

`skills/src` 파일들이 왜 단독 실행이 아닌지, 액션·컨테이너가 뭔지 정리한다.

## 한눈에 보기

| 용어 | 정체 | 예 |
| --- | --- | --- |
| 메시지 | 주고받는 데이터 틀 | `sensor_msgs/Image`, `Skill.action`의 goal/result |
| 액션 | 오래 걸리는 일용 요청 방식 (`goal → 피드백 → result` + 취소) | `/skills/pick` |
| 액션 서버 | 요청을 받아 실행하는 쪽 | skill 노드 (`serve()` 안에서 `create_server`) |
| 액션 클라이언트 | 요청을 보내는 쪽 | `coordinator` (`create_client`) |
| 노드 | ROS 그래프상의 실행 단위(이름) | `skill_pick`, `coordinator` |
| 컴포넌트 | 프로세스에 올리는 플러그인 형태 노드 (독립 실행 파일 없음) | `skills::PickSkill` |
| 컨테이너 | 컴포넌트를 묶어 돌리는 프로세스 (도커 아님) | `arm_skills`, `base_skills` |

## skill은 왜 `main`·`spin`이 없나

skill 클래스들은 `SkillNode`를 상속한 **컴포넌트 플러그인**이라 단독 실행 파일이 없다.
실행·스핀은 컨테이너 프로세스가 맡는다.

- 공통 틀: `lv2_module6/ros2_ws/src/skills/include/skills/skill_node.hpp:24` (`SkillNode : public rclcpp::Node`)
- 예: `PickSkill`은 생성자에서 `/skills/pick` 서버만 등록한다: `lv2_module6/ros2_ws/src/skills/src/pick_skill.cpp:26`
- 빌드는 공유 라이브러리 하나 + 플러그인 등록뿐이다: `lv2_module6/ros2_ws/src/skills/CMakeLists.txt:32`

## 프로세스 배치는 2개

`bringup`이 `skills.launch.py`를 항상 포함한다 (`src/bringup/launch/bringup.launch.py:197`).
OS 프로세스는 2개, ROS 노드 이름은 11개다.

```text
arm_skills  (component_container)
  skill_pose, skill_inspect, skill_pick, skill_place,
  skill_load, skill_unload, skill_stand

base_skills (component_container)
  skill_navigate, skill_approach
```

## 호출은 coordinator가 직접 고른다

분배기 노드는 따로 없다. `coordinator`가 계획 단계(`SubGoal.skill`)에 적힌
이름대로 `/skills/<skill>` 클라이언트를 순서대로 호출한다.

- 클라이언트 생성: `lv2_module6/ros2_ws/src/plan/src/coordinator.cpp:96` (`/skills/<액션>` 10개)
- 호출: `Coordinator::send` → `async_send_goal`: `lv2_module6/ros2_ws/src/plan/src/coordinator.cpp:318`
- 예: `pose` 단계는 `send("pose", ...)` 후 끝나면 다음 단계: `lv2_module6/ros2_ws/src/plan/src/coordinator.cpp:283`
- 서버가 아직 없으면 1초 뒤 같은 단계를 다시 시도한다: `lv2_module6/ros2_ws/src/plan/src/coordinator.cpp:293`

## 통신 방식: 기본은 DDS 3종

 intra-process on/off가 바꿀 수 있다:

1. **일반 ROS 통신 (지금 쓰는 방식)** — 토픽·서비스·액션을 DDS로 주고받는다.
   액션도 속을 뜯으면 토픽+서비스 조합이다. skill끼리는 같은 프로세스 안이라도
   이렇게 돈다 (intra-process 미설정).
2. **프로세스 내 통신 (zero-copy)** — 같은 프로세스 안 노드끼리 DDS·직렬화를
   건너뛰고 메시지 포인터를 바로 넘긴다. 켜는 법:
   `ComposableNode(..., extra_arguments=[{'use_intra_process_comms': True}])`
   또는 `NodeOptions::use_intra_process_comms(true)`. 진짜 복사 방지는
   `publish()`에 `unique_ptr`를 넘길 때만 된다.
3. **직접 C++ 호출·공유 메모리** — 기술적으로는 되지만 실행 순서·스레드와
   꼬이기 쉬워서 권장하지 않는다.

on/off로 바뀌는 건 **전달 방식**뿐이다. 사용자 코드가 만지는 인터페이스는
어느 쪽이든 토픽·서비스·액션 그대로다.

## 확인할 때

```bash
ros2 node list | grep -E 'skill|arm_skills|base_skills'
ros2 component list /arm_skills
ros2 component list /base_skills
ros2 action list | grep /skills/
ros2 action info /skills/pick
```
