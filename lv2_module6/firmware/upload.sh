#!/bin/bash
# OpenCR 펌웨어 업로드만 (컴파일하지 않음)
#
#   ./upload.sh [포트] [파일.bin|파일.opencr]
#
#   포트 기본: /dev/opencr, 없으면 /dev/ttyACM0
#   파일 기본: firmware/build/ 아래에서 가장 최근 *.ino.bin
#
# .bin은 ROBOTIS opencr_ld_shell make로 .opencr 헤더를 붙인 뒤 업로드한다.
# 설치: ./setup.sh, 컴파일: ./compile.sh, 한 번에: ./setup_compile_upload.sh
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

need_uploader

port=${1:-$(default_port)}
artifact=${2:-}

if [ -z "$artifact" ]; then
  artifact=$(find "$BUILD_DIR" -type f -name '*.ino.bin' -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -1 | cut -d' ' -f2- || true)
  [ -n "$artifact" ] || {
    echo "FAIL: 컴파일된 펌웨어 없음 ($BUILD_DIR/*/*.ino.bin) → ./compile.sh 먼저 실행"
    exit 1
  }
fi

artifact=$(cd "$(dirname "$artifact")" && pwd)/$(basename "$artifact")
[ -f "$artifact" ] || { echo "FAIL: 파일 없음: $artifact"; exit 1; }
[ -e "$port" ] || { echo "FAIL: 포트 없음: $port (ls -l /dev/opencr /dev/ttyACM*)"; exit 1; }

# ROS controller_manager가 같은 포트를 잡고 있으면 업로드가 깨진다
if pgrep -f ros2_control_node >/dev/null; then
  echo "FAIL: ros2_control_node(controller_manager) 실행 중 → ROS(start.sh)를 먼저 종료하세요."
  exit 1
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

case "$artifact" in
  *.opencr)
    cp "$artifact" "$work/upload.opencr"
    ;;
  *.bin)
    name=$(basename "$artifact")
    name=${name%.ino.bin}
    name=${name%.bin}
    cp "$artifact" "$work/$name.bin"
    echo "=== .bin → .opencr ($artifact)"
    (cd "$work" && "$LD_SHELL" make "$name.bin" "$name" "V$(date +%y%m%d%H%M)")
    [ -f "$work/$name.opencr" ] || { echo "FAIL: .opencr 변환 실패"; exit 1; }
    ;;
  *)
    echo "FAIL: .bin 또는 .opencr 파일만 지원: $artifact"
    exit 1
    ;;
esac

if [ -f "$work/upload.opencr" ]; then
  opencr="$work/upload.opencr"
else
  opencr="$work/$name.opencr"
fi

echo "=== upload: $artifact → $port"
# 보드가 응답 안 하면 opencr_ld_shell은 무한 대기 → 시간 제한
timeout 120 "$LD_SHELL" "$port" 115200 "$opencr" 1 || {
  echo "FAIL: 업로드 실패/응답 없음 ($port)"
  echo "  - 포트가 OpenCR가 맞는지 (ls -l /dev/opencr /dev/ttyACM*)"
  echo "  - 권한(dialout 그룹), ROS/시리얼 모니터가 포트를 잡고 있는지 확인"
  echo "  - OpenCR의 SW2를 누른 채 RESET → 부트로더 모드에서 다시 실행"
  exit 1
}
echo "OK: 업로드 완료 ($artifact → $port)"
