#!/bin/bash
# bring-up 검사 (컨테이너 안에서 실행): test-bringup
#   1. colcon 빌드
#   2. bringup launch (use_ros_system:=false — 이미지에 📦 패키지가 없음)
#   3. 🛠 노드가 전부 떴고, 패키지 config/*.yaml에 적은 파라미터가 노드에 실제로 있고,
#      /diagnostics_agg (diagnostic_aggregator)에 노드마다 상태가 모이면 PASS
#      (yaml 오타는 ROS가 조용히 무시하고 기본값으로 돈다 → 여기서 잡는다. 메시지 흐름은 test-faker)
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }

KILL_PAT='(/ws/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch'   # ROS 노드 전부 + launch (패키지를 적으면 새 패키지가 빠져 고아로 남음)

stop_nodes() {
  pkill -f "$KILL_PAT" 2>/dev/null || return 0
  for _ in $(seq 20); do
    pgrep -f "$KILL_PAT" >/dev/null || return 0
    sleep 0.5
  done
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
}
stop_nodes  # 이전 실행의 고아 노드가 있으면 DDS 그래프가 꼬이니 먼저 정리 (빌드 전에 — 남은 노드가 돌면 빌드도 느려짐)

build-ws || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash

# ML 노드는 도커에서 띄우지 않는다 (GPU·모델 없음 — faker ML 대역이 대신, ml 패키지는 test-packages로 검사)
NODES="/perception /user_input /vlm_task_planner /coordinator /explorer /arm_safety_monitor /manual_control /body_filter /state_manager"

# job control: 백그라운드 launch가 SIGINT를 무시하지 않게 (ros2 launch는 SIGINT로 정상 종료)
set -m
ros2 launch bringup bringup.launch.py use_ros_system:=false use_ml:=false > /tmp/bringup.log 2>&1 & LAUNCH_PID=$!
UP=""
PARAMS=""
for _ in $(seq 20); do  # 노드 기동 + DDS discovery 대기 (최대 20초)
  sleep 1
  UP=$(ros2 node list 2>/dev/null)
  missing=0
  for n in $NODES; do echo "$UP" | grep -qx "$n" || missing=1; done
  [ "$missing" = 0 ] && break
done
# yaml의 "노드: ros__parameters: 키"마다 노드에 선언돼 있는지 (노드 하나당 ros2 param list 한 번)
PARAMS=$(python3 - <<'PY'
import glob, subprocess, yaml
bad = []
for f in sorted(glob.glob('/ws/install/*/share/*/config/*.yaml')):
    if '/ros_system/' in f or '/faker/' in f or '/ml/' in f:   # ros_system은 📦·컨트롤러, faker는 test-faker, ml은 도커에서 안 띄움
        continue
    def flat(d, prefix=''):  # 중첩 파라미터 → 점으로 이은 이름 (analyzers.perception.type 등)
        for k, v in d.items():
            if isinstance(v, dict):
                yield from flat(v, f'{prefix}{k}.')
            else:
                yield prefix + k
    for node, body in (yaml.safe_load(open(f)) or {}).items():
        keys = list(flat((body or {}).get('ros__parameters', {})))
        # diagnostics.yaml 최상위 키(analyzers)가 곧 노드 이름 (aggregator_node 바이너리는 analyzers로 뜬다)
        out = subprocess.run(['ros2', 'param', 'list', '/' + node], capture_output=True, text=True).stdout
        have = {l.strip() for l in out.splitlines()}
        bad += [f'{f}: /{node} {k}' for k in keys if k not in have]
print('\n'.join(bad))
PY
)
# 노드별 상태가 aggregator에 모였는지 (diagnostic_updater는 1초마다 발행)
AGG=$(timeout 15 ros2 topic echo --once --filter 'len(m.status) > 10' /diagnostics_agg diagnostic_msgs/msg/DiagnosticArray 2>/dev/null)
kill -INT "$LAUNCH_PID" 2>/dev/null
timeout 15 tail --pid="$LAUNCH_PID" -f /dev/null
stop_nodes

echo "--- nodes ---"
echo "$UP"
for n in $NODES; do
  echo "$UP" | grep -qx "$n" || { echo "FAIL: 노드 없음 $n"; tail -n 40 /tmp/bringup.log; exit 1; }
done
for n in $NODES; do
  echo "$AGG" | grep -q "name: .*${n#/}: " || { echo "FAIL: /diagnostics_agg에 ${n#/} 상태 없음"; echo "$AGG" | grep "name:" | head -30; exit 1; }
done
[ -z "$PARAMS" ] || { echo "FAIL: config yaml에 있는데 노드에 없는 파라미터 (오타?)"; echo "$PARAMS"; exit 1; }
echo "PASS: bringup (🛠 노드 전부 기동 · config yaml 파라미터 일치 · /diagnostics_agg 상태)"
