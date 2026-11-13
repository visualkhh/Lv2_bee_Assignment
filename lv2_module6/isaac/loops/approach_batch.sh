#!/usr/bin/env bash
# Repeat approach_loop.py N times on a fresh scene each round (reset.sh between rounds) and summarise the docking error
# against the TRUE box (pucks ground truth). Run while isaac/standalone/isaac_sim.sh <scene> is up and playing.
#
#   ./approach_batch.sh 10 1                 # 10 rounds, start box 1 (extra args go to approach_loop.py)
#   ./approach_batch.sh 10 1 --plan-error 0.04 -0.02
#
# A reset whose stack does not come up (lifecycle transition stalled — seen 2 of 4 startups on 11-12) is retried
# up to 3 times. Summary: logs/approach_batch_<ts>.txt (success = position error <= 10 mm; yaw reported separately).
HERE="$(cd "$(dirname "$0")" && pwd)"
N="${1:?rounds}"; BOX="${2:?start box}"; shift 2
RESET="$HERE/../standalone/runtime/reset.sh"
DIRS=()
for i in $(seq "$N"); do
  for try in 1 2 3; do
    echo "=== round $i/$N (reset try $try)"
    "$RESET" >/dev/null || exit 1
    sleep 15  # ROS stop -> Stop/Play -> ROS start
    python3 "$HERE/approach_loop.py" "$BOX" --wait-ready 150 "$@"
    rc=$?
    [ $rc -ne 3 ] && break
  done
  [ $rc -eq 3 ] && { echo "round $i: stack never came up — skipped"; continue; }
  DIRS+=("$(ls -td "$HERE"/logs/approach_* | grep -v batch | head -1)")
done
"$HERE/approach_summary.py" "${DIRS[@]}" | tee "$HERE/logs/approach_batch_$(date +%m%d_%H%M%S).txt"
