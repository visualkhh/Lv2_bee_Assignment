# 퍽 이송 로봇(beebot) — ROS 2 실전 노드 설계

미로 코스에서 퍽을 찾아 목표 구역으로 옮기고 장애물을 피하는(자율주행 기반 물류 이송 및 장애물 회피) TurtleBot3 Waffle Pi + OpenMANIPULATOR-X(+손목) + pole_front_cam·gripper_cam(RealSense) 모바일 매니퓰레이터의 아키텍처·노드 설계 문서입니다. 구현: [lv2_module6](../../../lv2_module6/README.md).
VLM이 계획하고, Nav2·MoveIt이 확정 실행하고, 학습 모델은 앞에서 자문·뒤에서 보정만 합니다.

- 원본(인터랙티브: 노드 클릭·흐름 강조): <https://dooboostore.github.io/ros2?concept=design&sec=arch> — 원본은 "컵 가져오기" 예시 버전이고, 이 문서는 우리 미션(퍽 이송·장애물 회피)과 실제 구현(lv2_module6)에 맞게 고쳤다
- 원본 소스: `dooboostore.github.io/apps/center/src/pages/ros2/components/RosDesign.ts`, `RosDesignNodes.ts`
- 도식은 mermaid 코드 블록 — GitHub·IntelliJ markdown 미리보기에서 그대로 렌더됩니다.

## 목차

1. [개요·원칙](00-overview.md)
2. [전체 아키텍처](01-arch.md)
3. [1 센서·인지](02-sense.md)
4. [2 계획·임무](03-plan.md)
5. [3 Nav2·SLAM](04-nav.md)
6. [3-1 지도 맵핑 실습](04b-mapping.md)
7. [4 팔·MoveIt](05-arm.md)
8. [5 베이스](06-base.md)
9. [6 TF·QoS](07-tf.md)
10. [ros2_control 연결](08-ctrl.md)
11. [직접작성·제공](09-nodes.md)
12. [노드 사전](10-catalog.md)
13. [학습 설계](11-learn.md)
14. [Isaac 파이프라인](12-isaac.md)
15. [시나리오](13-scenario.md)
16. [체크리스트·로드맵](14-check.md)
17. [검토 반영](15-changes.md)
18. [API 명세서(토픽·서비스·액션)](../API.md)

## 도식 읽는 법

| 구분 | 의미 |
| --- | --- |
| 색 | 초록 = 확정 실행 · 주황 = 학습(RL·지도학습) · 보라 = VLM · 빨강 = 안전 · 노랑 = 하드웨어 |
| 모양 | 사각형 = ROS 노드·컨트롤러 · 육각형 = 하드웨어(토픽 없음, 🔌로 드라이버에 연결) · 이중원 = 사람 · 점선 알약 = 다른 섹션과 잇는 연결점(← 들어옴 · → 나감, 숫자 = 섹션 번호) · 원통 = 파일·데이터셋 · 큰 영역 = 계층·섹터·프로세스 경계 |
| 노드 표시 | 🛠️ 직접 작성 · 📦 제공 패키지(ROS 2·Nav2·MoveIt·ros2_control 등) |
| 선 | 실선 = 토픽·데이터 흐름 · 굵은 선 = 액션 · 점선 = 서비스·파라미터·피드백·매핑 단계 전용 · 화살촉 없음 = 체인 결합(같은 프로세스) · 화살표 방향 = 데이터 방향 |
| 라벨 | ✉️ 토픽 · 🎬 액션 · 📞 서비스 · 🔌 시리얼·USB · 🧩 ros2_control 내부 인터페이스 · 💾 파일 · ⌨️ 사람 → 로봇 · ⌨️ 키보드 · 💬 로봇 → 사람 — 형식: **이모지 이름 (메시지 타입)** |
