#!/bin/bash
# CI 배포 패키지(ros2_ws-deploy-*.tar.gz) 내려받기 — wget만 사용, 로그인·gh 불필요
#
#   ./download-artifact.sh                           # 받을 수 있는 RUN ID 목록
#   ./download-artifact.sh 37416810439               # ./ros2_ws-37416810439/ 에 풀기
#   ./download-artifact.sh 37416810439 -o ~/deploy   # 풀 위치 지정 (비어 있는 폴더여야 함)
#   ./download-artifact.sh 37416810439 -t github_pat_...   # 토큰을 바로 넘김 (아래 '토큰 만들기')
#   ./download-artifact.sh 37416810439 -a aarch64          # 다른 아키텍처용 받기 (기본: 이 장치 uname -m)
#   Ubuntu 24.04 arm64(Jetson Orin Nano)는 ros2_ws-deploy-aarch64-noble.tar.gz를 자동 선택한다.
#
# 받는 곳 (둘 다 wget):
#   1) 토큰이 있으면 → Actions artifact를 Authorization 헤더로 받음 (모든 성공 실행·브랜치, 7일 보관)
#      Actions artifact는 공개 저장소여도 인증 없이는 401 → 토큰 필요
#   2) 없으면 → GitHub Release(태그 run-<RUN ID>)에서 인증 없이 받음 (main에 병합된 실행만 올라감)
#        https://github.com/<저장소>/releases/download/run-<RUN ID>/ros2_ws-deploy-aarch64-noble.tar.gz
#
# ── 토큰 만들기 (최초 1회, GitHub 웹) ───────────────────────────────────────
#   1. GitHub 오른쪽 위 프로필 사진 → Settings
#   2. 왼쪽 맨 아래 Developer settings → Personal access tokens → Fine-grained tokens
#   3. Generate new token
#        Token name        : lv2-artifact (아무 이름)
#        Expiration        : 30 days (짧게)
#        Repository access : Public repositories   ← 공개 저장소 읽기 전용. 다른 권한 선택 불필요
#   4. Generate token → github_pat_... 로 시작하는 값 복사 (이 화면을 벗어나면 다시 볼 수 없음)
#   (팀원도 각자 자기 계정으로 만들면 됨 — 저장소가 공개라 소유자가 아니어도 됨)
#
# ── 토큰 넘기는 방법 (우선순위: -t > GITHUB_TOKEN > 파일) ───────────────────
#   0. 실행할 때 바로:  ./download-artifact.sh <RUN ID> -t github_pat_...
#      명령 기록(~/.bash_history)에 남으므로 앞에 공백 한 칸을 붙여 실행하면 기록되지 않음 (Ubuntu 기본 설정)
#   A. 파일에 저장 (권장 — 이 스크립트가 자동으로 읽음)
#        mkdir -p ~/.config/lv2_bee
#        install -m 600 /dev/null ~/.config/lv2_bee/github_token      # 본인만 읽기 (먼저 권한부터)
#        read -rsp '토큰 붙여넣기: ' t && echo "$t" > ~/.config/lv2_bee/github_token; echo   # 화면에 안 보임
#   B. 환경변수 (지금 터미널에서만)
#        export GITHUB_TOKEN=github_pat_...
#   확인: ./download-artifact.sh --check-token
#   삭제: rm ~/.config/lv2_bee/github_token  (GitHub 토큰 페이지에서 Revoke도)
#
#   ⚠️ 토큰을 이 스크립트나 저장소 파일에 적지 않는다 — 공개 저장소에 올라가면 유출되고 GitHub이 자동 폐기함.
# RUN ID: Actions 실행 페이지 주소 …/actions/runs/<RUN ID> 의 숫자
#
# 풀린 결과: install/ · firmware/(build/*.bin 포함) · *.sh
#   - firmware·스크립트는 Raspberry Pi에서 바로 사용 가능 (./firmware-upload.sh)
#   - CI가 x86_64·aarch64용을 따로 만든다. 이 장치의 OS·아키텍처에 맞는 것을 받는다 (-a로 아키텍처 지정 가능)
#     파일: ros2_ws-deploy-x86_64.tar.gz (PC) · ros2_ws-deploy-aarch64.tar.gz (Ubuntu 26.04 ARM)
#           ros2_ws-deploy-aarch64-noble.tar.gz (Ubuntu 24.04 / Jetson Orin Nano)
set -euo pipefail

REPO="${REPO:-visualkhh/Lv2_bee_Assignment}"
ARCH="$(uname -m)"
# 패키지를 고를 때 쓰는 대상 OS. uname -r(커널)이 아니라 /etc/os-release의 Ubuntu 버전을 쓴다.
OS_ID="$(. /etc/os-release 2>/dev/null && echo "${ID:-linux}")"
OS_VERSION="$(. /etc/os-release 2>/dev/null && echo "${VERSION_ID:-unknown}")"
OS_TARGET="${OS_ID}-${OS_VERSION}"
TARGET_OVERRIDE="${TARGET:-}"
TOKEN_FILE="${TOKEN_FILE:-$HOME/.config/lv2_bee/github_token}"
# 응답이 없으면 무한 대기하지 않게: 30초 무응답이면 재시도 1번 후 실패
WGET=(wget --timeout=30 --tries=2)
API="https://api.github.com/repos/$REPO/actions"

usage() { awk 'NR > 1 && !/^#/ {exit} NR > 1' "$0"; }

list_releases() {
  echo "받을 수 있는 RUN ID ($REPO, 최근 순):"
  "${WGET[@]}" -qO- "https://api.github.com/repos/$REPO/releases?per_page=20" | python3 -c '
import json, sys
found = False
for r in json.load(sys.stdin):
    if r["tag_name"].startswith("run-") and any(a["name"] == sys.argv[1] for a in r["assets"]):
        print("  %s  %s  %s" % (r["tag_name"][4:], r["created_at"], r["name"]))
        found = True
if not found:
    print("  (없음 — main에 병합되어 CI가 성공하면 생김. 다른 브랜치 실행은 GITHUB_TOKEN으로 artifact 받기)")
' "$FILE"
  echo "→ ./download-artifact.sh <RUN ID>"
}

token_guide() {
  cat <<EOF

── 토큰이 필요해요 (Actions artifact는 공개 저장소여도 로그인 필요) ──
1) 토큰 만들기 (GitHub 로그인 상태에서, 값은 만들 때 한 번만 보임)
     https://github.com/settings/personal-access-tokens/new
     (메뉴: 프로필 사진 → Settings → Developer settings → Personal access tokens → Fine-grained tokens)
     Token name: 아무 이름 · Expiration: 30 days · Repository access: Public repositories
     → Generate token → github_pat_... 복사
2) 넘기기 (셋 중 하나)
     ./download-artifact.sh ${run_id:-<RUN ID>} -t github_pat_...      # 앞에 공백 한 칸 → 명령 기록에 안 남음
     export GITHUB_TOKEN=github_pat_...
     파일에 저장 (한 번 저장하면 다음부터 자동으로 읽음):
       mkdir -p "\$(dirname $TOKEN_FILE)" && install -m 600 /dev/null $TOKEN_FILE
       read -rsp '토큰 붙여넣기: ' t && echo "\$t" > $TOKEN_FILE; echo
3) 확인: ./download-artifact.sh --check-token
   잃어버림·만료: https://github.com/settings/personal-access-tokens 에서 삭제 후 새로 만들기
EOF
}

check_token() {
  [ -n "${GITHUB_TOKEN:-}" ] || { echo "토큰 없음"; token_guide; exit 1; }
  echo "토큰 확인 중..."
  if "${WGET[@]}" -qO /dev/null --header "Authorization: Bearer $GITHUB_TOKEN" https://api.github.com/user; then
    echo "ok: 토큰 유효"
  else
    echo "FAIL: 토큰이 틀렸거나 만료됨 — 새로 만들어 다시 저장"
    token_guide
    exit 1
  fi
}

run_id=''
dest=''
token_arg=''
do_check=0
while (($#)); do
  case "$1" in
    --check-token) do_check=1; shift ;;
    -t | --token) token_arg=$2; shift 2 ;;
    -o) dest=$2; shift 2 ;;
    -a) ARCH=$2; shift 2 ;;
    -h | --help) usage; exit 0 ;;
    -*) echo "모르는 옵션: $1"; usage; exit 2 ;;
    *) run_id=$1; shift ;;
  esac
done

# 아키텍처·OS별 패키지 이름. 예전 실행(아키텍처 구분 전)은 x86_64 빌드뿐이라 x86_64일 때만 예전 이름도 찾는다
if [ -n "$TARGET_OVERRIDE" ]; then
  TARGET_PACKAGE="$TARGET_OVERRIDE"
elif [ "$ARCH" = aarch64 ] && [ "$OS_TARGET" = ubuntu-24.04 ]; then
  TARGET_PACKAGE="ubuntu-24.04-aarch64"
else
  TARGET_PACKAGE="$OS_TARGET-$ARCH"
fi

case "$TARGET_PACKAGE" in
  ubuntu-24.04-aarch64) FILE="ros2_ws-deploy-aarch64-noble.tar.gz" ;;
  *) FILE="ros2_ws-deploy-$ARCH.tar.gz" ;;
esac
ARTIFACT_NAMES=("$FILE")
[ "$ARCH" = x86_64 ] && ARTIFACT_NAMES+=(ros2_ws-deploy.tar.gz ros2_ws-deploy)

# 토큰: -t → 환경변수 GITHUB_TOKEN → TOKEN_FILE
if [ -n "$token_arg" ]; then
  GITHUB_TOKEN=$token_arg
elif [ -z "${GITHUB_TOKEN:-}" ] && [ -r "$TOKEN_FILE" ]; then
  GITHUB_TOKEN=$(tr -d '[:space:]' < "$TOKEN_FILE")
fi
if [ "$do_check" = 1 ]; then
  check_token
  exit 0
fi

if [ -z "$run_id" ]; then
  list_releases
  exit 0
fi
[[ $run_id =~ ^[0-9]+$ ]] || { echo "RUN ID는 숫자: $run_id"; exit 2; }

dest=${dest:-"$PWD/ros2_ws-$run_id"}
if [ -e "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
  echo "FAIL: $dest 가 비어 있지 않음 (덮어쓰지 않음). 지우거나 -o로 다른 위치 지정"
  exit 1
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

if [ -n "${GITHUB_TOKEN:-}" ]; then
  # 1) Actions artifact (Authorization 헤더)
  auth=(--header "Authorization: Bearer $GITHUB_TOKEN" --header "Accept: application/vnd.github+json")
  echo "[1/4] 실행 $run_id 의 artifact 찾는 중..."
  listing=$("${WGET[@]}" -qO- "${auth[@]}" "$API/runs/$run_id/artifacts") \
    || { echo "FAIL: GitHub API 거부 — 토큰이 틀렸거나 만료됨, 또는 RUN ID 오류 ($run_id)"; token_guide; exit 1; }
  artifact_id=$(python3 -c '
import json, sys
for a in json.load(sys.stdin)["artifacts"]:
    if a["name"] in sys.argv[1:] and not a["expired"]:
        print(a["id"], a["size_in_bytes"]); break
' "${ARTIFACT_NAMES[@]}" <<< "$listing" || true)
  [ -n "$artifact_id" ] || { echo "FAIL: 실행 $run_id 에 ros2_ws-deploy artifact 없음 (실패한 실행·보관 기간 지남·토큰 권한)"; exit 1; }
  read -r artifact_id artifact_size <<< "$artifact_id"
  echo "      → artifact $artifact_id ($((artifact_size / 1024 / 1024))MB)"
  # API는 302로 저장소 서버(서명된 URL)로 보낸다. wget은 --header를 리다이렉트에도 붙여 보내서
  # 서명 URL이 거부하므로, 리다이렉트 주소만 먼저 받고 그 주소는 인증 헤더 없이 받는다.
  echo "[2/4] 다운로드 주소 요청 중..."
  response=$("${WGET[@]}" -S --max-redirect=0 -O /dev/null "${auth[@]}" "$API/artifacts/$artifact_id/zip" 2>&1 || true)
  status=$(awk '/^  HTTP\//{code=$2} END{print code}' <<< "$response")
  location=$(awk '/^  Location: /{print $2}' <<< "$response" | tail -1 | tr -d '\r')
  if [ -z "$location" ]; then
    case "$status" in
      401) why="토큰이 틀렸거나 만료됨" ;;
      403) why="토큰 권한 부족 — Fine-grained: Repository access를 Public repositories(또는 이 저장소 + Actions: Read)로" ;;
      404) why="artifact 없음 (RUN ID 확인)" ;;
      410) why="artifact 보관 기간(7일) 지남" ;;
      *) why="예상 못 한 응답" ;;
    esac
    echo "FAIL: 다운로드 주소를 못 받음 (HTTP ${status:-?}) — $why"
    [ "$status" = 401 ] || [ "$status" = 403 ] && token_guide
    exit 1
  fi
  echo "[3/4] 내려받는 중..."
  "${WGET[@]}" -q --show-progress --progress=bar:force -O "$work/download" "$location" \
    || { echo "FAIL: 저장소 서버에서 받기 실패 (서명 URL 만료 — 다시 실행)"; exit 1; }
  # zip('PK')으로 감싸져 오면 풀어서 tar.gz를 꺼낸다
  if [ "$(head -c 2 "$work/download" | od -An -tx1 | tr -d ' ')" = 504b ]; then
    (cd "$work" && python3 -m zipfile -e download . && rm -f download)
  else
    mv "$work/download" "$work/$FILE"
  fi
  tarball=$(find "$work" -name '*.tar.gz' | head -1 || true)
  [ -n "$tarball" ] || { echo "FAIL: 받은 파일에 tar.gz가 없음"; ls -la "$work"; exit 1; }
  [ "$tarball" = "$work/$FILE" ] || mv "$tarball" "$work/$FILE"
else
  # 2) Release (인증 없음)
  url="https://github.com/$REPO/releases/download/run-$run_id/$FILE"
  echo "=== 받기: $url"
  echo "[3/4] Release에서 내려받는 중... (토큰 없음)"
  "${WGET[@]}" -q --show-progress --progress=bar:force -O "$work/$FILE" "$url" || {
    echo "FAIL: Release에 없음 — 토큰 없이 받을 수 있는 건 main에 병합된 실행뿐 (RUN ID 오류·실패한 실행일 수도)"
    echo "      기능 브랜치·PR 실행($run_id 포함)은 토큰으로 artifact를 받아야 함"
    token_guide
    exit 1
  }
fi

# 최상위 폴더 없이 묶여 있음. (혹시 ros2_ws/ 로 묶인 파일이면 한 단계 벗긴다)
# (pipefail에서 head가 tar를 SIGPIPE로 끊으므로 첫 항목을 먼저 받아 둔다)
first=$(tar -tzf "$work/$FILE" | head -1 || true)
strip=()
[[ $first == ros2_ws/* ]] && strip=(--strip-components=1)
echo "[4/4] 푸는 중... → $dest"
mkdir -p "$dest"
tar -xzf "$work/$FILE" -C "$dest" "${strip[@]}"

echo "=== 풀린 위치: $dest"
ls -l "$dest"
bins=("$dest"/firmware/build/*/*.ino.bin)
if [ -e "${bins[0]}" ]; then
  echo "펌웨어: ${bins[*]}"
fi
pkg_arch=$(cat "$dest/ARCH" 2>/dev/null || echo x86_64)   # ARCH 파일이 없으면 예전(x86_64) 패키지
if [ "$pkg_arch" != "$(uname -m)" ]; then
  echo "NOTE: 이 패키지의 install/은 $pkg_arch 빌드 → 이 장치($(uname -m))에선 실행 불가 (펌웨어·스크립트는 사용 가능)"
fi
pkg_target=$(cat "$dest/TARGET" 2>/dev/null || true)
if [ -n "$pkg_target" ] && [ "$pkg_target" != "$TARGET_PACKAGE" ]; then
  echo "NOTE: 이 패키지의 TARGET은 $pkg_target → 이 장치($TARGET_PACKAGE)와 OS/ROS가 다를 수 있음"
  echo "      Ubuntu 24.04 Jetson은 RUN ID를 새로 받아 ros2_ws-deploy-aarch64-noble.tar.gz를 선택해야 함"
fi
