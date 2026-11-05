# tool/node — 노드별 관리 도구

노드마다 폴더 하나. 그 노드의 **기록(rosbag2)** · **분석(report.png)** · 진단을 한 곳에 모은다.
공통 bag 읽기·궤적 계산은 `common/bagio.py`.

```text
tool/node/
├── README.md
├── common/
│   └── bagio.py                    # rosbag2 읽기 · 경로 투영(횡오차) · yaw
└── rl_nav_controller/              # Nav2 주행 컨트롤러 (정책 vs DWB)
    ├── run
    ├── record.sh
    └── report.py                   # 횡오차·평균속도·정책 행동
```

## rl_nav_controller

정책(ONNX) vs inner DWB를 같은 주행으로 비교한다.

```bash
cd tool/node/rl_nav_controller
./run record ~/bags/policy          # enable=true (toggle true)
./run record ~/bags/dwb             # toggle false
./run report ~/bags/policy ~/bags/dwb --labels 정책 DWB --out ~/report.png
```

토글:

```bash
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"
```

## 노드 추가 시

1. `tool/node/<노드>/` 폴더
2. `record.sh` — 그 노드가 내는 토픽·디버그 토픽
3. `report.py` — 지표 정의(예: arm은 관절 추종오차·전류, controller는 정책 행동·횡오차)
4. `run` — `record|report` 서브커맨드
5. 이 README 표에 한 줄
