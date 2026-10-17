#!/bin/bash
# 실기 실행: 환경 확인 → (필요하면 빌드) → bringup launch
#
#   ./start.sh                              # 📦(카메라·EKF·RTAB-Map·controller_manager) + 🛠 노드 전부, OpenCR 실기
#   ./start.sh mock_hardware:=true          # OpenCR 없이 (명령을 상태로 되돌림 — launch 인자는 그대로 전달)
#   ./start.sh port:=/dev/ttyACM1           # OpenCR 포트 지정 (기본 /dev/opencr — udev 규칙 99-opencr.rules)
#   ./start.sh learning_enabled:=false      # 학습 노드 끄고 (비교 기준)
#   ./start.sh map:=slam                    # 지도 매핑 (새 폴더 map/<날짜_시각>/) — 기본 map:=saved = 저장 지도로 위치추정
#   ./start.sh --build [launch 인자]        # 저장소에서 실행할 때 colcon build 후 실행
#
# 두 가지 위치에서 동작한다.
#   - 저장소 lv2_module6/ros2_ws/ : install/이 없거나 --build면 src/를 빌드
#   - CI 배포 패키지(ros2_ws-deploy) : 이미 빌드된 install/을 그대로 사용 (펌웨어 업로드는 firmware-upload.sh)
# NOTE: set -u 사용 금지 — ROS setup.bash가 미설정 변수를 참조해서 죽음.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-lyrical}"

build=0
if [ "${1:-}" = --build ]; then
  build=1
  shift
fi
case "${1:-}" in
  -h | --help) sed -n '2,10p' "$0"; exit 0 ;;
esac

# --- ROS 환경
ros_setup="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
[ -f "$ros_setup" ] || { echo "FAIL: ROS 2 $ROS_DISTRO_NAME 없음 ($ros_setup)"; exit 1; }
# shellcheck disable=SC1090
source "$ros_setup"

# --- 빌드 (저장소에서 실행할 때만)
if [ "$build" = 1 ] || [ ! -f "$HERE/install/setup.bash" ]; then
  [ -d "$HERE/src" ] || { echo "FAIL: install/도 src/도 없음 ($HERE)"; exit 1; }
  echo "=== colcon build ($HERE)"
  (cd "$HERE" && colcon build --symlink-install)
fi
# shellcheck disable=SC1091
source "$HERE/install/setup.bash"
# Fast DDS TypeLookup 끔 — 노드 40개에서 UDP 폭주로 controller_manager가 못 뜨는 것 방지 (설명: ros_system/config/fastdds.xml)
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HERE/install/ros_system/share/ros_system/config/fastdds.xml}"

# --- 아키텍처 확인: 다른 CPU용으로 빌드된 install/이면 노드가 Exec format error로 죽으므로 미리 멈춘다
# (예: CI x86_64 패키지를 Raspberry Pi에서 실행). 실행 파일의 ELF 헤더 e_machine(오프셋 18)으로 판정.
elf=''
while IFS= read -r f; do   # 실행 파일 중 첫 ELF (파이썬 스크립트 등은 건너뜀)
  if [ "$(head -c 4 "$f" | od -An -c | tr -d ' ')" = '177ELF' ]; then elf=$f; break; fi
done < <(find "$HERE/install" -path '*/lib/*' -type f -perm -u+x 2>/dev/null)
if [ -n "$elf" ]; then
  case "$(od -An -tx1 -j18 -N2 "$elf" | tr -d ' \n')" in
    3e00) bin_arch=x86_64 ;;
    b700) bin_arch=aarch64 ;;
    *) bin_arch=unknown ;;
  esac
  if [ "$bin_arch" != unknown ] && [ "$bin_arch" != "$(uname -m)" ]; then
    echo "FAIL: install/은 $bin_arch용 빌드인데 이 장치는 $(uname -m) — 실행 불가 (Exec format error)"
    echo "      - CI 패키지라면: ./download-artifact.sh <RUN ID> 로 $(uname -m)용(ros2_ws-deploy-$(uname -m).tar.gz)을 받기"
    echo "      - 저장소라면:    ./start.sh --build  로 이 장치에서 다시 빌드"
    exit 1
  fi
fi

# --- 장치 확인 (경고만 — 장치 없이 실행하는 경우도 있음)
mock=false
use_cameras=true
port=/dev/opencr
for arg in "$@"; do
  case "$arg" in
    mock_hardware:=true) mock=true ;;
    use_cameras:=false) use_cameras=false ;;
    port:=*) port=${arg#port:=} ;;
  esac
done

if [ "$mock" = false ]; then
  # BeebotSystemHardware가 여는 포트 (URDF port → ros_system 인자 port)
  if [ ! -e "$port" ]; then
    echo "WARN: OpenCR 포트 없음: $port — USB 연결 확인 (ls /dev/ttyACM*) · udev 규칙: sudo cp $(dirname "$0")/99-opencr.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger · 없이 시험하려면 mock_hardware:=true"
  elif [ ! -w "$port" ]; then
    echo "WARN: $port 쓰기 권한 없음 → sudo usermod -aG dialout \$USER (재로그인)"
  else
    echo "ok: OpenCR $port"
  fi
fi

if [ "$use_cameras" = true ]; then
  # grep -q는 pipefail에서 앞 명령이 SIGPIPE로 실패할 수 있어 출력을 먼저 받아 둔다
  if command -v rs-enumerate-devices >/dev/null; then
    devices=$(rs-enumerate-devices -s 2>/dev/null || true)
    count=$(grep -c "Intel RealSense" <<<"$devices" || true)
    if [ "$count" -ge 2 ]; then
      echo "ok: RealSense ${count}대 (pole_cam · gripper_cam)"
    else
      echo "WARN: RealSense ${count}대 — 2대(pole_cam · gripper_cam) 필요 (USB 3 포트·udev 규칙 확인)"
    fi
  else
    echo "WARN: rs-enumerate-devices 없음 — RealSense 연결 확인 생략"
  fi
fi

# 지도 폴더 기본 = 이 워크스페이스의 map/ (테스트베드 /ws/map 과 같은 자리) — map:=slam(새 폴더 매핑) · saved(저장 지도, 기본)
case " $* " in *" map_dir:="*) ;; *) set -- "$@" "map_dir:=$HERE/map" ;; esac
echo "=== ros2 launch bringup bringup.launch.py $*"
exec ros2 launch bringup bringup.launch.py "$@"
