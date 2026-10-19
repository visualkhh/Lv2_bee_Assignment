#!/bin/bash
# OpenCR 펌웨어 컴파일 + 업로드 (Raspberry Pi Ubuntu arm64 / x86_64 PC)
#
#   ./upload.sh --setup              # 최초 1회: 툴 설치 (sudo 필요)
#   ./upload.sh [포트] [스케치폴더]    # 컴파일 → .opencr 변환 → 업로드
#                                     #   포트 기본 /dev/opencr (udev), 없으면 /dev/ttyACM0, 스케치 기본 이 폴더의 유일한 스케치
#
# 왜 arduino-cli upload가 아닌가:
#   ROBOTIS 보드 인덱스의 업로더(opencr_ld)·컴파일러(opencr_gcc)는 x86/Win/Mac 전용 → ARM엔 없음.
#   - 컴파일: Ubuntu 시스템 arm-none-eabi-gcc를 툴체인 자리에 연결 (docker/Dockerfile arm64 우회와 동일)
#   - 업로드: ROBOTIS가 Pi용으로 배포하는 opencr_ld_shell (TurtleBot3 opencr_update) 사용.
#             .bin을 그대로 못 올리고 `make`로 .opencr(이름·버전 헤더 추가)로 변환 후 업로드.
#   opencr_ld_shell_arm은 32비트 ARM 바이너리 → 64비트 Ubuntu에선 libc6:armhf 필요.
set -euo pipefail

FQBN=OpenCR:OpenCR:OpenCR
BOARD_URL=https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
OPENCR_VERSION=1.5.3
OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c
OPENCR_UPDATE_URL=https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
TOOLS_DIR="$HOME/.local/share/opencr_update"   # opencr_ld_shell 설치 위치
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$HERE/build"
SUDO=$([ "$(id -u)" = 0 ] || echo sudo)

case "$(uname -m)" in
  aarch64 | armv7l) ARCH=arm; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_arm" ;;
  x86_64) ARCH=x86; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_x86" ;;
  *) echo "지원 안 하는 아키텍처: $(uname -m)"; exit 1 ;;
esac

setup() {
  echo "=== apt 패키지"
  local pkgs=(curl ca-certificates bzip2)
  if [ "$ARCH" = arm ]; then
    pkgs+=(gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib)
    # 64비트 Ubuntu에서 32비트 ARM 업로더를 돌리기 위한 armhf libc
    [ "$(uname -m)" = aarch64 ] && $SUDO dpkg --add-architecture armhf
    pkgs+=(libc6:armhf)
  fi
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends "${pkgs[@]}"

  echo "=== arduino-cli"
  if ! command -v arduino-cli >/dev/null; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
      | $SUDO env BINDIR=/usr/local/bin sh
  fi
  arduino-cli config init --overwrite
  arduino-cli config add board_manager.additional_urls "$BOARD_URL"
  arduino-cli core update-index

  echo "=== OpenCR 보드 코어"
  local data
  data=$(arduino-cli config get directories.data)
  if [ "$ARCH" = x86 ]; then
    arduino-cli core install OpenCR:OpenCR
  elif [ ! -d "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" ]; then
    # 보드 패키지만 수동 설치 + 시스템 gcc를 opencr_gcc 자리에 연결
    # NOTE: 릴리스 파일명이 .tar.bz2지만 내용은 gzip → xzf
    local tmp
    tmp=$(mktemp -d)
    curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/$OPENCR_VERSION/opencr.tar.bz2" -o "$tmp/opencr.tar.bz2"
    echo "$OPENCR_SHA256  $tmp/opencr.tar.bz2" | sha256sum -c -
    tar xzf "$tmp/opencr.tar.bz2" -C "$tmp"
    mkdir -p "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" \
      "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
    cp -a "$tmp/opencr/." "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION/"
    ln -sf /usr/bin/arm-none-eabi-* "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
    rm -rf "$tmp"
  fi
  arduino-cli lib install Dynamixel2Arduino

  echo "=== opencr_ld_shell (ROBOTIS opencr_update)"
  # NOTE: 이것도 이름만 .tar.bz2고 내용은 gzip → 파일로 받아 tar가 형식을 판별하게 (xf)
  mkdir -p "$TOOLS_DIR"
  curl -fsSL "$OPENCR_UPDATE_URL" -o "$TOOLS_DIR/opencr_update.tar"
  tar xf "$TOOLS_DIR/opencr_update.tar" -C "$TOOLS_DIR" --strip-components=1 \
    opencr_update/opencr_ld_shell_arm opencr_update/opencr_ld_shell_x86
  rm -f "$TOOLS_DIR/opencr_update.tar"
  chmod +x "$TOOLS_DIR"/opencr_ld_shell_*

  arduino-cli board details --fqbn "$FQBN" >/dev/null && echo "OK: setup 완료 ($ARCH)"
  echo "시리얼 권한이 없으면: sudo usermod -aG dialout \$USER  (재로그인 필요)"
}

upload() {
  local port=${1:-$([ -e /dev/opencr ] && echo /dev/opencr || echo /dev/ttyACM0)}
  local sketch=${2:-}
  if [ -z "$sketch" ]; then
    local found=("$HERE"/*/*.ino)
    [ "${#found[@]}" = 1 ] && [ -e "${found[0]}" ] \
      || { echo "스케치를 지정하세요: ./upload.sh $port <스케치폴더>"; exit 1; }
    sketch=$(dirname "${found[0]}")
  fi
  sketch=$(cd "$sketch" && pwd)
  local name out
  name=$(basename "$sketch")
  out="$BUILD_DIR/$name"

  [ -x "$LD_SHELL" ] && command -v arduino-cli >/dev/null \
    || { echo "툴 없음 → 먼저 ./upload.sh --setup"; exit 1; }
  [ -e "$port" ] || { echo "포트 없음: $port (ls /dev/ttyACM* 로 확인, USB 연결?)"; exit 1; }
  # ROS의 controller_manager(BeebotSystemHardware)가 같은 포트를 잡고 있으면 업로드가 깨진다
  if pgrep -f ros2_control_node >/dev/null; then
    echo "ros2_control_node(controller_manager) 실행 중 → 포트 충돌. ROS(start.sh)를 먼저 종료하세요."; exit 1
  fi

  echo "=== compile: $sketch"
  rm -rf "$out"
  mkdir -p "$out"
  arduino-cli compile --fqbn "$FQBN" --library Dynamixel2Arduino --output-dir "$out" "$sketch"

  echo "=== .bin → .opencr"
  (cd "$out" && "$LD_SHELL" make "$name.ino.bin" "$name" "V$(date +%y%m%d%H%M)")

  echo "=== upload: $out/$name.opencr → $port"
  # 보드가 응답 안 하면 opencr_ld_shell은 무한 대기 → 시간 제한 (정상 업로드는 수 초)
  timeout 120 "$LD_SHELL" "$port" 115200 "$out/$name.opencr" 1 || {
    echo "FAIL: 업로드 실패/응답 없음 ($port)"
    echo "  - 포트가 OpenCR가 맞는지 (ls /dev/ttyACM*), 권한(dialout 그룹)"
    echo "  - 안 되면 OpenCR의 SW2(PUSH SW2)를 누른 채 RESET → 부트로더 모드에서 다시 실행"
    exit 1
  }
  echo "OK: 업로드 완료 ($name → $port)"
}

if [ "${1:-}" = --setup ]; then
  setup
else
  upload "$@"
fi
