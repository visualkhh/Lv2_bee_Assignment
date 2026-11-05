#!/usr/bin/env bash
# 모델 파일 내려받기 — Notion/Drive/일반 URL → 이 워크스페이스 model/ 아래.
#
#   ./download.sh '<URL>' [받을폴더] [파일이름] [--name 파일이름] [--unzip] [--sha256 해시]
#   주의: URL은 반드시 따옴표로 감싼다 (Notion 링크의 &가 셸에서 잘림)
#
# 예)
#   ./download.sh 'https://.../policy.onnx?download=1' rl_nav_correction
#   ./download.sh 'https://.../policy.onnx?download=1' rl_nav_correction policy.onnx
#   ./download.sh 'https://.../policy.onnx?download=1' nav_controller
#   ./download.sh 'https://.../nav.zip?download=1' nav_controller --unzip
#   ./download.sh 'https://.../policy.onnx' rl_nav_correction --sha256 3f2a...
#
# 받을폴더를 생략하면 model/ 바로 아래에 저장한다. 상대 경로는 이 스크립트 위치(model/) 기준.
# 파일이름을 생략하면 URL 경로의 마지막 조각을 쓴다 (기존 동작).
# 기본 동작: 이어받기(-C -) · 리다이렉트(-L) · HTTP 오류면 실패 · 브라우저 User-Agent(Notion 403 회피).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
URL=""
DEST=""
NAME=""
UNZIP=0
SHA=""

usage() { sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }

if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then usage; exit 0; fi
URL="$1"; shift
# 두/세 번째 위치 인자: [받을폴더] [파일이름]
if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
  if [ $# -gt 1 ] && [[ "$2" != --* ]]; then
    DEST="$1"; NAME="$2"; shift 2
  elif [[ "$1" =~ \.(onnx|data|zip|tar\.gz|tgz|pt|bin|json|yaml|yml)$ ]]; then
    NAME="$1"; shift            # 파일이름만 준 경우 → 기본 폴더(model/)
  else
    DEST="$1"; shift            # 받을폴더만 준 경우
  fi
fi
while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="${2:?--name 값}"; shift 2 ;;
    --name=*) NAME="${1#--name=}"; shift ;;
    --unzip) UNZIP=1; shift ;;
    --sha256) SHA="${2:?--sha256 값}"; shift 2 ;;
    --sha256=*) SHA="${1#--sha256=}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "FAIL: 모르는 인자: $1" >&2; usage >&2; exit 2 ;;
  esac
done

command -v curl >/dev/null || { echo "FAIL: curl 없음 (sudo apt install curl)" >&2; exit 2; }

# 따옴표를 안 써서 & 뒤가 잘린 흔한 실수 감지 (Notion 서명 URL은 signature=가 있어야 완전)
case "$URL" in
  *file.notion.com*|*notion.so*|*notion.site*)
    [[ "$URL" == *"signature="* ]] || echo "WARN: URL에 signature=가 없습니다 — 따옴표 없이 & 뒤가 잘렸을 수 있습니다 (URL을 '...'로 감싸세요)" >&2 ;;
esac

# 저장 폴더 — 상대 경로는 model/ 기준
if [ -z "$DEST" ]; then
  DEST="$HERE"
elif [[ "$DEST" != /* ]]; then
  DEST="$HERE/$DEST"
fi
mkdir -p "$DEST"

# 파일 이름 — --name > URL 경로 마지막 조각 > 기본
if [ -z "$NAME" ]; then
  NAME="$(printf '%s' "${URL%%\?*}" | sed 's|/$||' | awk -F/ '{print $NF}')"
  # URL 인코딩(%20 등)과 흔한 다운로드 경로 이름 보정
  NAME="$(printf '%b' "${NAME//%/\\x}" 2>/dev/null || printf '%s' "$NAME")"
  [ -n "$NAME" ] && [ "$NAME" != "download" ] || NAME="model.bin"
fi
OUT="$DEST/$NAME"

echo "URL  : $URL"
echo "저장 : $OUT"

# 이미 있으면 크기 확인 후 이어받기 (같으면 curl이 바로 끝난다)
curl -fL --retry 3 --retry-delay 2 -C - \
  -A 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124 Safari/537.36' \
  -o "$OUT" "$URL"

[ -s "$OUT" ] || { echo "FAIL: 받은 파일이 비었습니다: $OUT" >&2; rm -f "$OUT"; exit 1; }

if [ -n "$SHA" ]; then
  got="$(sha256sum "$OUT" | awk '{print $1}')"
  if [ "$got" != "$SHA" ]; then
    echo "FAIL: sha256 불일치" >&2
    echo "  기대: $SHA" >&2
    echo "  실제: $got" >&2
    exit 1
  fi
  echo "sha256 OK"
fi

if [ "$UNZIP" = 1 ]; then
  case "$OUT" in
    *.zip)
      command -v unzip >/dev/null || { echo "FAIL: unzip 없음 (sudo apt install unzip)" >&2; exit 2; }
      echo "압축 해제: $DEST"
      unzip -o "$OUT" -d "$DEST" >/dev/null
      echo "  → $(ls -1 "$DEST" | tr '\n' ' ')"
      ;;
    *.tar.gz|*.tgz)
      tar -xzf "$OUT" -C "$DEST"
      echo "  → $(ls -1 "$DEST" | tr '\n' ' ')"
      ;;
    *) echo "WARN: --unzip인데 압축 파일이 아님: $NAME" >&2 ;;
  esac
fi

# ONNX 외부 가중치(.onnx.data) 누락 경고 — 파일 안에 데이터 경로 문자열이 있는데 옆에 없으면
case "$OUT" in
  *.onnx)
    if grep -aq '\.onnx\.data\|\.data' "$OUT" 2>/dev/null && [ ! -f "$OUT.data" ]; then
      echo "WARN: 이 ONNX는 외부 가중치를 씁니다 → 같은 폴더에 $(basename "$OUT").data 도 받아야 합니다" >&2
      echo "      ./download.sh '<...data URL>' $(basename "$DEST") --name $(basename "$OUT").data" >&2
    fi
    ;;
esac

echo "OK: $(du -h "$OUT" | cut -f1)  $OUT"
echo "    모델 경로 예: ros2_ws/model/$(basename "$DEST")/$NAME"
echo "    (rl_nav_correction은 model_path: policy.onnx → rl_nav_correction/, rl_nav_controller는 nav_controller/)"
