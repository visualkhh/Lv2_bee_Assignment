#!/usr/bin/env bash
# Train skills one after another (one run per GPU), each followed by a deterministic eval; one summary line per skill.
#   isaac/scripts/train_queue.sh                       # every skill below
#   isaac/scripts/train_queue.sh dock_box load_slot    # only these (split across machines)
#   ITERS=1500 ENVS=512 EVAL=1024 isaac/scripts/train_queue.sh ...
# Summary: logs/queue_summary.tsv (skill, policy file, zero-residual success, policy success). Run it detached from SSH
# (systemd-run --user ... or a desktop terminal): an SSH hang-up kills the Isaac app.
cd "$(dirname "$(realpath "$0")")/.." || exit 1  # isaac/: logs/ · policies/ 기준
PY=${PY:-$HOME/pa-course/IsaacLab-6.1/.venv/bin/python}
ITERS=${ITERS:-1500}; ENVS=${ENVS:-512}; EVAL=${EVAL:-1024}
XARGS=${XARGS:-}  # extra train.py args for every run, e.g. XARGS=--sag_dr
SUB=${SUB:-}  # policy subfolder under policies/, e.g. SUB=v02 (queue summary goes to logs/queue_summary_$SUB.tsv)
[ -n "$SUB" ] && SUB="$SUB/"
declare -A TASK=(  # skills.yaml name -> train.py task and cfg
  [place_cell]="pip -" [pick_standing_puck]="pick -" [dock_box]="drive task=dock" [drive_straight]="drive task=straight" [park_home]="drive task=park" [drive_arc]="drive task=arc"
  [load_slot]="load -" [unload_slot]="unload -" [pregrasp]="reach target=above" [regrasp_after_stand]="stand regrasp=1" [stand_lying_puck]="stand -" [pick_standing_puck]="pick -"
  [regrasp_stood]="stand regrasp=1"  # old name (already sent to a colleague)
)
ORDER=(dock_box drive_straight load_slot unload_slot pregrasp park_home drive_arc regrasp_after_stand)
[ $# -gt 0 ] && ORDER=("$@")
mkdir -p logs policies/$SUB
SUM=logs/queue_summary${SUB:+_${SUB%/}}.tsv
[ -f $SUM ] || printf "skill\tpolicy\tzero\tpolicy_success\tfinished\n" > $SUM
rate() { grep -E "^\[$1\] episodes" "$2" | tail -1 | sed -E 's/.*= ([0-9.]+) %.*/\1/'; }
for s in "${ORDER[@]}"; do
  read -r task cfg <<< "${TASK[$s]}"
  [ -z "$task" ] && { echo "unknown skill $s"; continue; }
  c=(); [ "$cfg" != "-" ] && c=(--cfg "$cfg")
  log=logs/queue_$s.log
  echo "[queue] $s: zero" | tee $log
  $PY scripts/train.py --task $task "${c[@]}" --mode zero $XARGS --num_envs 256 --episodes $EVAL >> $log 2>&1
  echo "[queue] $s: train $ENVS envs x $ITERS" | tee -a $log
  $PY scripts/train.py --task $task "${c[@]}" --mode train $XARGS --num_envs $ENVS --iters $ITERS --run $s --out ${SUB}$s.pt >> $log 2>&1
  echo "[queue] $s: eval" | tee -a $log
  $PY scripts/train.py --task $task "${c[@]}" --mode eval $XARGS --num_envs 256 --episodes $EVAL --policy policies/${SUB}$s.pt >> $log 2>&1
  printf "%s\tpolicies/%s%s.pt\t%s\t%s\t%s\n" "$s" "$SUB" "$s" "$(rate zero $log)" "$(rate policy $log)" "$(date +%m-%d_%H:%M)" | tee -a $SUM
done
