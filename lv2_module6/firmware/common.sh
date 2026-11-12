# firmware 공통 변수·함수 (source 해서 사용)
# 이 파일은 직접 실행하지 않는다.

COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HERE="$COMMON_DIR"
BUILD_DIR="$HERE/build"

FQBN=OpenCR:OpenCR:OpenCR
BOARD_URL=https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
OPENCR_VERSION=1.5.3
OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c
OPENCR_UPDATE_URL=https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
TOOLS_DIR="$HOME/.local/share/opencr_update"
SUDO=$([ "$(id -u)" = 0 ] || echo sudo)

case "$(uname -m)" in
  aarch64 | armv7l) ARCH=arm; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_arm" ;;
  x86_64) ARCH=x86; LD_SHELL="$TOOLS_DIR/opencr_ld_shell_x86" ;;
  *) echo "지원 안 하는 아키텍처: $(uname -m)"; return 1 2>/dev/null || exit 1 ;;
esac

default_port() {
  if [ -e /dev/opencr ]; then echo /dev/opencr; else echo /dev/ttyACM0; fi
}

need_arduino() {
  if ! command -v arduino-cli >/dev/null 2>&1; then
    if [ -x "$HOME/bin/arduino-cli" ]; then
      export PATH="$HOME/bin:$PATH"
    else
      echo "FAIL: arduino-cli 없음 → ./setup.sh 먼저 실행" >&2
      return 1
    fi
  fi
}

need_uploader() {
  [ -x "$LD_SHELL" ] || {
    echo "FAIL: OpenCR 업로더 없음 ($LD_SHELL) → ./setup.sh 먼저 실행" >&2
    return 1
  }
}
