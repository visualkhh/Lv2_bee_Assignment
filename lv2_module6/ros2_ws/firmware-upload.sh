#!/bin/bash
# OpenCR 펌웨어 업로드 (배포 패키지·저장소 공용 진입점)
#
#   ./firmware-upload.sh --setup       # 툴 설치 (firmware/setup.sh 호출)
#   ./firmware-upload.sh [포트]        # 업로드
#
#   포트 기본: /dev/opencr, 없으면 /dev/ttyACM0
#
# 동작:
#   - CI 배포 패키지(./firmware/ 존재): 미리 컴파일된 .bin을 ./firmware/upload.sh로 업로드
#   - 저장소(../firmware/ 존재): ../firmware/setup_compile_upload.sh로 설치·컴파일·업로드
#
# 개별 단계는 firmware/의 setup.sh · compile.sh · upload.sh를 직접 사용한다.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"

if [ -x "$ROOT/firmware/setup.sh" ]; then
  FW="$ROOT/firmware"
  PACKAGE=1
elif [ -x "$ROOT/../firmware/setup.sh" ]; then
  FW="$(cd "$ROOT/../firmware" && pwd)"
  PACKAGE=0
else
  echo "FAIL: firmware/ 또는 ../firmware/ 를 찾을 수 없음"
  exit 1
fi

case "${1:-}" in
  -h | --help)
    sed -n '2,13p' "$0"
    exit 0
    ;;
  --setup)
    exec "$FW/setup.sh"
    ;;
esac

port=${1:-$([ -e /dev/opencr ] && echo /dev/opencr || echo /dev/ttyACM0)}

if [ "$PACKAGE" = 1 ]; then
  bins=("$FW"/build/*/*.ino.bin)
  [ -e "${bins[0]}" ] || {
    echo "FAIL: 컴파일된 펌웨어 없음 ($FW/build/*/*.ino.bin)"
    echo "      패키지에서 직접 컴파일하려면: $FW/setup_compile_upload.sh"
    exit 1
  }
  [ "${#bins[@]}" = 1 ] || { echo "FAIL: .bin이 여러 개: ${bins[*]}"; exit 1; }
  exec "$FW/upload.sh" "$port" "${bins[0]}"
else
  # 저장소: 한 번에 설치·컴파일·업로드
  exec "$FW/setup_compile_upload.sh" "$port"
fi
