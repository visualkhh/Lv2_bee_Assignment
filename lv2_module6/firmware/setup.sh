#!/bin/bash
# OpenCR 툴 설치 (최초 1회 또는 환경이 바뀌었을 때)
#
#   ./setup.sh
#
# 설치:
#   - arduino-cli, OpenCR 보드 코어, Dynamixel2Arduino
#   - ARM: 시스템 arm-none-eabi-gcc + ROBOTIS opencr_ld_shell
#   - x86: OpenCR 코어의 opencr_gcc를 시스템 gcc 링크로 교체
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

echo "=== apt 패키지 ($ARCH)"
pkgs=(curl ca-certificates bzip2)
if [ "$ARCH" = arm ]; then
  pkgs+=(gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib)
  [ "$(uname -m)" = aarch64 ] && $SUDO dpkg --add-architecture armhf
  pkgs+=(libc6:armhf)
fi
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends "${pkgs[@]}"

echo "=== arduino-cli"
if ! command -v arduino-cli >/dev/null 2>&1; then
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | $SUDO env BINDIR=/usr/local/bin sh
fi
# 사용자 ~/bin에 이미 설치한 경우도 인식
[ -x "$HOME/bin/arduino-cli" ] && export PATH="$HOME/bin:$PATH"

arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls "$BOARD_URL"
arduino-cli core update-index

echo "=== OpenCR 보드 코어"
data=$(arduino-cli config get directories.data)
if [ "$ARCH" = x86 ]; then
  arduino-cli core install OpenCR:OpenCR
  # ROBOTIS 32비트 툴체인은 64비트 Ubuntu에서 실행 불가 → 시스템 gcc 링크
  rm -rf "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  mkdir -p "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  for f in /usr/bin/arm-none-eabi-*; do
    ln -sf "$f" "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
  done
elif [ ! -d "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" ]; then
  # ARM: 보드 패키지만 수동 설치 + 시스템 gcc 링크
  # NOTE: 릴리스 파일명이 .tar.bz2지만 내용은 gzip → xzf
  tmp=$(mktemp -d)
  curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/$OPENCR_VERSION/opencr.tar.bz2" \
    -o "$tmp/opencr.tar.bz2"
  echo "$OPENCR_SHA256  $tmp/opencr.tar.bz2" | sha256sum -c -
  tar xzf "$tmp/opencr.tar.bz2" -C "$tmp"
  mkdir -p "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" \
    "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  cp -a "$tmp/opencr/." "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION/"
  ln -sf /usr/bin/arm-none-eabi-* "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
  rm -rf "$tmp"
fi

arduino-cli lib install Dynamixel2Arduino

echo "=== opencr_ld_shell ($ARCH)"
# 이름만 .tar.bz2이고 내용은 gzip → 파일로 받아 tar가 형식을 판별
mkdir -p "$TOOLS_DIR"
curl -fsSL "$OPENCR_UPDATE_URL" -o "$TOOLS_DIR/opencr_update.tar"
tar xf "$TOOLS_DIR/opencr_update.tar" -C "$TOOLS_DIR" --strip-components=1 \
  opencr_update/opencr_ld_shell_arm opencr_update/opencr_ld_shell_x86
rm -f "$TOOLS_DIR/opencr_update.tar"
chmod +x "$TOOLS_DIR"/opencr_ld_shell_*

arduino-cli board details --fqbn "$FQBN" >/dev/null
echo "OK: setup 완료 ($ARCH, $LD_SHELL)"
echo "시리얼 권한이 없으면: sudo usermod -aG dialout \$USER  (재로그인 필요)"
