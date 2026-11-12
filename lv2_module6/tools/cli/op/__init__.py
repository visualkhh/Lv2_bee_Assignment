"""op 패키지 — operator CLI의 OOP 분리.

- terminal: 터미널 원자 연산 (크기·색·너비)
- defs: 메뉴 내비게이션 상수
- ui: Ui — 프레임 그리기 + 방향키 선택 + 모달 입력 (화면 전부)
- state: StateStore — ROS 상태 보관 (값·장치·관절·진단)
- bus: Bus — 발행 + 전송 + 대기 (로봇 동작 전부)
- pages: Page(기반) → Menu(선택 루프) → 각 메뉴 화면
"""
