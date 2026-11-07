#!/bin/bash
# OpenCR 펌웨어 컴파일만 (업로드하지 않음)
#
#   ./compile.sh [스케치폴더]
#
# 스케치 기본값: 이 폴더 아래의 유일한 *.ino
# 출력: firmware/build/<스케치>/<스케치>.ino.bin
#
# 업로드는 ./upload.sh, 설치부터 한 번에 하려면 ./setup_compile_upload.sh
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

need_arduino

sketch=${1:-}
if [ -z "$sketch" ]; then
  found=("$HERE"/*/*.ino)
  [ "${#found[@]}" = 1 ] && [ -e "${found[0]}" ] \
    || { echo "FAIL: 스케치를 지정하세요: ./compile.sh <스케치폴더>"; exit 1; }
  sketch=$(dirname "${found[0]}")
fi
sketch=$(cd "$sketch" && pwd)
name=$(basename "$sketch")
out="$BUILD_DIR/$name"

rm -rf "$out"
mkdir -p "$out"

echo "=== compile: $sketch"
arduino-cli compile --fqbn "$FQBN" --library Dynamixel2Arduino \
  --output-dir "$out" "$sketch"

bin="$out/$name.ino.bin"
[ -f "$bin" ] || { echo "FAIL: 컴파일 결과 없음: $bin"; exit 1; }
echo "OK: compile 완료 ($bin)"
