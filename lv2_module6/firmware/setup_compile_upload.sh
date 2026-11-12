#!/bin/bash
# OpenCR 펌웨어 설치 + 컴파일 + 업로드를 한 번에
#
#   ./setup_compile_upload.sh [포트] [스케치폴더]
#
#   포트 기본: /dev/opencr, 없으면 /dev/ttyACM0
#   스케치 기본: 이 폴더 아래의 유일한 *.ino
#
# 개별 실행:
#   ./setup.sh       # 툴 설치
#   ./compile.sh     # .bin 생성
#   ./upload.sh      # .bin/.opencr 업로드
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/common.sh"

port=${1:-$(default_port)}
sketch=${2:-}

echo "########## 1/3 setup ##########"
"$HERE/setup.sh"

echo "########## 2/3 compile ##########"
if [ -n "$sketch" ]; then
  "$HERE/compile.sh" "$sketch"
else
  "$HERE/compile.sh"
fi

echo "########## 3/3 upload ##########"
"$HERE/upload.sh" "$port"
