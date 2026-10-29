#!/bin/bash
# 실기 실행: 환경 확인 → (필요하면 빌드) → bringup launch
#
#   ./start.sh                              # 📦(카메라·EKF·RTAB-Map·controller_manager) + 🛠 노드 전부, OpenCR 실기
#   ./start.sh mock_hardware:=true          # OpenCR 없이 (명령을 상태로 되돌림 — launch 인자는 그대로 전달)
#   ./start.sh port:=/dev/ttyACM1           # OpenCR 포트 지정 (기본 /dev/opencr — udev 규칙 99-opencr.rules)
#   POLE_FRONT_CAM_SERIAL=... GRIPPER_CAM_SERIAL=... ./start.sh  # RealSense serial 고정 (권장)
#   RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL=WARN ./start.sh  # 로그 소음 줄이기 (기본 INFO — 전 노드 적용)
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
  -h | --help) sed -n '2,11p' "$0"; exit 0 ;;
esac

# --- 패키지 대상 OS 확인 (CI 배포 패키지에만 TARGET 파일이 있음)
# Ubuntu 26.04용 install/을 Ubuntu 24.04 Jetson에서 실행하면 glibc/Python/ROS가 달라
# 아키텍처가 같아도 죽는다. start.sh가 먼저 명확히 막는다.
if [ -f "$HERE/TARGET" ]; then
  pkg_target=$(cat "$HERE/TARGET")
  host_target="$(. /etc/os-release 2>/dev/null; echo "${ID:-linux}-${VERSION_ID:-unknown}-$(uname -m)")"
  if [ "$pkg_target" != "$host_target" ]; then
    echo "FAIL: 이 패키지는 $pkg_target용인데 이 장치는 $host_target — 실행 불가"
    echo "      Jetson(Ubuntu 24.04)은 ros2_ws-deploy-aarch64-noble.tar.gz를 받아야 함"
    echo "      다시 받기: ./download-artifact.sh <RUN ID>  (장치 OS·아키텍처를 자동 선택)"
    exit 1
  fi
fi

# --- ROS 환경
ros_setup="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
[ -f "$ros_setup" ] || { echo "FAIL: ROS 2 $ROS_DISTRO_NAME 없음 ($ros_setup)"; exit 1; }
# shellcheck disable=SC1090
source "$ros_setup"

# nvblox(isaac_ros_nvblox)를 별도 workspace에 소스 빌드한 경우 overlay를 자동으로 올린다 (Jetson arm64용 apt가 없음).
#   ~/isaac_ros_ws/install/setup.bash — documents/jetson-orin-nano/CUVSLAM.md "nvblox"
# 없는 경우에도 계속 진행하고, SLAM을 띄울 때 ros_system check_slam이 무엇이 없는지 알려 준다.
SLAM_WS="${SLAM_WS:-$HOME/isaac_ros_ws}"
if [ -f "$SLAM_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$SLAM_WS/install/setup.bash"
fi

# --- 빌드 (저장소에서 실행할 때만)
if [ "$build" = 1 ] || [ ! -f "$HERE/install/setup.bash" ]; then
  [ -d "$HERE/src" ] || { echo "FAIL: install/도 src/도 없음 ($HERE)"; exit 1; }
  echo "=== colcon build ($HERE)"
  (cd "$HERE" && colcon build --symlink-install)
fi
# shellcheck disable=SC1091
source "$HERE/install/setup.bash"
# Fast DDS TypeLookup 끔 — 노드 40개에서 UDP 폭주로 controller_manager가 못 뜨는 것 방지 (설명: fastdds.xml)
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HERE/fastdds.xml}"
[ -f "$FASTDDS_DEFAULT_PROFILES_FILE" ] || echo "WARN: Fast DDS 프로파일 없음: $FASTDDS_DEFAULT_PROFILES_FILE — 기본 전송으로 실행 (UDP 폭주 가능)"
# 실기에서 확인한 함정 — 이 둘이면 Nav2 수명주기 서비스 응답이 버려져 velocity_smoother·collision_monitor가 안 켜짐 (텔레오퍼 안 움직임).
# 기기 안 격리는 fastdds_local.xml(SHM만)로 — documents/jetson-orin-nano/SETTING.md 5
[ -n "${ROS_DISCOVERY_SERVER:-}" ] && echo "WARN: ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER — 서비스 응답 timeout으로 Nav2가 안 켜질 수 있음 (격리는 fastdds_local.xml)"
[ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" = LOCALHOST ] || [ "${ROS_LOCALHOST_ONLY:-0}" = 1 ] && echo "WARN: LOCALHOST discovery — 노드 일부가 서로 안 보일 수 있음 (격리는 fastdds_local.xml + SUBNET)"
echo "Fast DDS 프로파일: $FASTDDS_DEFAULT_PROFILES_FILE"

# 로그 소음 줄이기: RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL=WARN ./start.sh (기본 INFO — 전 노드 적용, rcl 표준).
# 우리 노드 중요 이벤트는 WARN 이상이라 WARN으로 올려도 운영에 지장 없음 (INFO 계획 상세·주기 상태는 안 나옴).
export RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL="${RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL:-INFO}"

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

# --- Discovery Server: ROS_DISCOVERY_SERVER가 자기 자신을 가리키면 서버도 여기서 띄운다.
#   127.0.0.1:11811·localhost:11811·자기IP:11811 → 로컬 서버 시작(없으면) 후 사용 (내가 서버 자리)
#   남의 IP → 그쪽 서버에 붙기만 함 (서버 안 띄움)
#   미설정 → 멀티캐스트 기본
if [ -n "${ROS_DISCOVERY_SERVER:-}" ]; then
  _ds_first="${ROS_DISCOVERY_SERVER%%;*}"
  _ds_host="${_ds_first%%:*}"
  _ds_port="${_ds_first##*:}"
  [ "$_ds_port" = "$_ds_first" ] && _ds_port=11811
  _ds_local=false
  case "$_ds_host" in
    127.0.0.1|localhost) _ds_local=true ;;
  esac
  if [ "$_ds_local" = false ]; then
    # shellcheck disable=SC2046
    for _ds_ip in $(hostname -I 2>/dev/null); do
      if [ "$_ds_ip" = "$_ds_host" ]; then _ds_local=true; break; fi
    done
  fi
  if [ "$_ds_local" = true ]; then
    if pgrep -f "fastdds discovery -i 0 -p ${_ds_port}" >/dev/null 2>&1; then
      echo "ok: discovery server already running (:${_ds_port})"
    else
      echo "=== fastdds discovery 시작 (:${_ds_port})"
      setsid fastdds discovery -i 0 -p "$_ds_port" > /tmp/ds.log 2>&1 < /dev/null &
    fi
  fi
  echo "Discovery: server ($ROS_DISCOVERY_SERVER)"
else
  echo "Discovery: multicast (기본)"
fi

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

rs_count=0
devices=''
if [ "$use_cameras" = true ]; then
  # grep -q는 pipefail에서 앞 명령이 SIGPIPE로 실패할 수 있어 출력을 먼저 받아 둔다
  if command -v rs-enumerate-devices >/dev/null; then
    devices=$(rs-enumerate-devices -s 2>/dev/null || true)
    count=$(grep -c "Intel RealSense" <<<"$devices" || true)
    rs_count=$count
    if [ "$count" -ge 2 ]; then
      echo "ok: RealSense ${count}대 (pole_front_cam · gripper_cam)"
    else
      echo "WARN: RealSense ${count}대 — 2대(pole_front_cam · gripper_cam) 필요 (USB 3 포트·udev 규칙 확인)"
    fi
  else
    echo "WARN: rs-enumerate-devices 없음 — RealSense 연결 확인 생략"
  fi
fi

# RealSense serial_no를 정한다.
#   우선순위: 명령줄 인자 > 환경변수
#   환경변수: POLE_FRONT_CAM_SERIAL · GRIPPER_CAM_SERIAL
# RealSense ROS 드라이버는 /dev 경로가 아니라 serial_no를 받는다.
front_present=0
gripper_present=0
front_env_serial=''
gripper_env_serial=''
for spec in \
  "pole_front_cam_serial:POLE_FRONT_CAM_SERIAL" \
  "gripper_cam_serial:GRIPPER_CAM_SERIAL"; do
  key=${spec%%:*}
  env_name=${spec#*:}

  set_by_user=0
  for a in "$@"; do
    case "$a" in "$key":=*) set_by_user=1 ;; esac
  done
  if [ "$set_by_user" = 1 ]; then
    case "$key" in pole_front_cam_serial) front_present=1 ;; gripper_cam_serial) gripper_present=1 ;; esac
    continue
  fi

  if [ -n "${!env_name:-}" ]; then
    set -- "$@" "$key:=${!env_name}"
    echo "ok: $key=${!env_name} (env $env_name)"
    case "$key" in
      pole_front_cam_serial) front_present=1; front_env_serial=${!env_name} ;;
      gripper_cam_serial) gripper_present=1; gripper_env_serial=${!env_name} ;;
    esac
  fi
done

# 환경변수에 남아 있는 serial이 실제로 감지되지 않으면 그 카메라를 끈다.
# (한 대만 꽂았는데 예전 GRIPPER_CAM_SERIAL이 남아 있는 경우가 대표적)
# 명령줄로 직접 준 serial은 사용자의 의도일 수 있으므로 건드리지 않는다.
if [ "$use_cameras" = true ] && [ "$rs_count" -gt 0 ]; then
  if [ -n "$front_env_serial" ] && ! grep -Fq "$front_env_serial" <<<"$devices"; then
    echo "WARN: POLE_FRONT_CAM_SERIAL=$front_env_serial 감지 안 됨 — use_pole_front_cam:=true를 직접 주지 않았으면 끔"
    case " $* " in
      *" use_pole_front_cam:="*) ;;
      *) set -- "$@" "use_pole_front_cam:=false"; front_present=0 ;;
    esac
  fi
  if [ -n "$gripper_env_serial" ] && ! grep -Fq "$gripper_env_serial" <<<"$devices"; then
    echo "WARN: GRIPPER_CAM_SERIAL=$gripper_env_serial 감지 안 됨 — use_gripper_cam:=true를 직접 주지 않았으면 끔"
    case " $* " in
      *" use_gripper_cam:="*) ;;
      *) set -- "$@" "use_gripper_cam:=false"; gripper_present=0 ;;
    esac
  fi
fi

# serial을 하나도 안 주면 RealSense 노드의 serial_no가 빈 문자열이 되어
# 두 노드가 같은 첫 장치를 잡을 수 있다. 자동으로 임의 serial을 넣지 않고 경고만 한다.
if [ "$use_cameras" = true ] && [ "$front_present" = 0 ] && [ "$gripper_present" = 0 ] \
  && [ -z "$front_env_serial" ] && [ -z "$gripper_env_serial" ]; then
  if [ "$rs_count" -ge 2 ]; then
    echo "WARN: RealSense 2대인데 serial 미지정 — 두 노드가 같은 장치를 잡을 수 있음"
    echo "      POLE_FRONT_CAM_SERIAL=... GRIPPER_CAM_SERIAL=... ./start.sh"
  elif [ "$rs_count" -eq 1 ]; then
    echo "WARN: RealSense 1대인데 serial 미지정 — 한쪽 카메라를 use_*_cam:=false로 끄거나 serial 지정"
  fi
fi

# 한쪽 serial만 설정됐으면 나머지 카메라 노드는 끈다 (한 대만 연결한 상태에서
# 빈 serial 노드가 같은 카메라를 잡으려는 충돌 방지). 직접 use_*_cam:= 인자를 준 경우는 존중한다.
if [ "$front_present" = 1 ] && [ "$gripper_present" = 0 ]; then
  case " $* " in
    *" use_gripper_cam:="*) ;;
    *) set -- "$@" "use_gripper_cam:=false"; echo "WARN: gripper_cam serial 없음 → use_gripper_cam:=false" ;;
  esac
fi
if [ "$gripper_present" = 1 ] && [ "$front_present" = 0 ]; then
  case " $* " in
    *" use_pole_front_cam:="*) ;;
    *) set -- "$@" "use_pole_front_cam:=false"; echo "WARN: pole_front_cam serial 없음 → use_pole_front_cam:=false" ;;
  esac
fi

# MoveIt이 설치되어 있지 않으면 launch 전체가 "package not found"로 죽는다.
# 명시적으로 use_moveit:=true/false를 준 경우는 존중하고, 아니면 스택이 뜨도록 false로 내린다.
if ! ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1; then
  case " $* " in
    *" use_moveit:="*) ;;
    *) set -- "$@" "use_moveit:=false" ;;
  esac
  echo "WARN: moveit_ros_move_group 없음 → use_moveit:=false (팔 pick/place는 사용 불가)"
fi

# 지도 폴더 기본 = 이 워크스페이스의 map/ (테스트베드 /ws/map 과 같은 자리) — map:=slam(새 폴더 매핑) · saved(저장 지도, 기본)
case " $* " in *" map_dir:="*) ;; *) set -- "$@" "map_dir:=$HERE/map" ;; esac
echo "=== ros2 launch bringup bringup.launch.py $*"
exec ros2 launch bringup bringup.launch.py "$@"
