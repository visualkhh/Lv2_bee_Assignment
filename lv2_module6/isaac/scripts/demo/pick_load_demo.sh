#!/usr/bin/env bash
# 집기 시연 환경 + ROS (SLAM·Nav2 없음): 바닥 · 시작 상자 · 퍽 2개 · 상자에서 30 cm 로봇 (manipulator_env.py)
#   isaac/scripts/demo/pick_load_demo.sh · 시연: python3 -u isaac/scripts/demo/pick_load_play.py --clear-basket · 리셋: isaac/scripts/standalone/reset.sh
#   인자는 manipulator_env.py로 (--headless 등) · --no-policy만 여기서: 그 skill 정책을 끔 (기준 동작만)
#     --no-policy pick_standing_puck,load_slot  (이름 = models/skills/ 폴더 · all = 전부)
#   --no-twist-state: joint2·3 상태 = 엔코더 그대로 (기본은 엔코더 + 중력 비틀림 = 실제 각, BeebotSystemHardware twist_state)
STANDALONE=$(dirname "$(realpath "$0")")/../standalone
# YOLO: ~/.venv-yolo의 torch·ultralytics만 얹음 — numpy·cv2는 시스템 것 (cv_bridge가 numpy 1.x, isaac_sim.sh YOLO_VENV 주석)
VENV=${YOLO_SOURCE_VENV:-$HOME/.venv-yolo}/lib/python3.12/site-packages
OVERLAY=$HOME/.local/share/beebot/yolo_overlay
if [ ! -d "$OVERLAY" ] && [ -d "$VENV" ]; then
  mkdir -p "$OVERLAY/lib/python3.12/site-packages"
  for f in "$VENV"/*; do
    case "$(basename "$f")" in
      numpy|numpy-*|numpy.libs|cv2|opencv*|__pycache__|pip|pip-*|setuptools*) ;;
      *) ln -s "$f" "$OVERLAY/lib/python3.12/site-packages/" ;;
    esac
  done
fi
export YOLO_VENV=$OVERLAY
export ISAACSIM_VENV=${MANIPULATOR_DEMO_ISAAC_VENV:-$HOME/pa-course/.venv-isaacsim}  # ~/.bashrc의 ISAACSIM_VENV는 Isaac Lab용 — 지금까지 시연은 이 venv
export ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py
export ISAAC_STANDALONE="$STANDALONE/manipulator_env.py"
ARGS=()
NO_POLICY=
while [ $# -gt 0 ]; do
  case "$1" in
    --no-policy) NO_POLICY=$2; shift 2 ;;
    --no-twist-state) export BEEBOT_TWIST_STATE=0; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
# 정책 끄기: 설치된 모델 폴더를 링크로 복사하고 고른 skill만 meta.yaml use: false → skill 노드 model_dir (isaac_manipulator_demo.launch.py)
if [ -n "$NO_POLICY" ]; then
  MODEL=/tmp/beebot_model
  rm -rf "$MODEL" && cp -rs "$(realpath "$STANDALONE/../../../ros2_ws/install/skills/share/skills/model")" "$MODEL" || exit 1
  [ "$NO_POLICY" = all ] && NO_POLICY=$(ls "$MODEL" | paste -sd,)
  for s in ${NO_POLICY//,/ }; do
    [ -f "$MODEL/$s/meta.yaml" ] || { echo "--no-policy: 정책 $s 없음 — $(ls "$MODEL" | paste -sd' ')"; exit 1; }
    sed 's/^use: true/use: false/' "$MODEL/$s/meta.yaml" > "$MODEL/$s/meta.tmp" && mv -f "$MODEL/$s/meta.tmp" "$MODEL/$s/meta.yaml"
  done
  export BEEBOT_MODEL_DIR=$MODEL
  echo "정책 끔: $NO_POLICY"
fi
export ISAAC_STANDALONE_ARGS="${ARGS[*]}"
exec "$STANDALONE/isaac_sim.sh"
