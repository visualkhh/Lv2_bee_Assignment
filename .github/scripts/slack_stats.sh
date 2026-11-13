#!/usr/bin/env bash
# Slack 통계 → README의 SLACK_STATS 블록 갱신 + main 커밋/push.
#
# 사용:
#   ./.github/scripts/slack_stats.sh --fetch
#       → slack_secrets.txt로 Slack에서 직접 긁어 통계 (자동/수동 공통)
#   ./.github/scripts/slack_stats.sh
#       → .github/scripts/slack.zip 을 읽는다 (fallback, gitignore됨)
#   ./.github/scripts/slack_stats.sh ~/Downloads/slackdump_20261113_195914.zip
#   ./.github/scripts/slack_stats.sh --no-push
#       → README만 갱신하고 커밋/push는 하지 않는다
#
# 자동: GitHub Actions .github/workflows/slack-stats.yml이 매일 09:30 KST에 실행.
# 이 스크립트는 로컬 수동 fallback이다 (`--fetch` 또는 덤프 ZIP).
# 매일 도는 contributors action은 CONTRIBUTORS 블록만 갱신하므로
# 이 스크립트가 만드는 SLACK_STATS 블록과 겹치지 않는다.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "$ROOT" ]; then
  echo "git 저장소 안에서 실행해라." >&2
  exit 2
fi
cd "$ROOT"

DUMP="$ROOT/.github/scripts/slack.zip"
PUSH=1
FETCH=0
for arg in "$@"; do
  case "$arg" in
    --no-push) PUSH=0 ;;
    --fetch) FETCH=1 ;;
    -h|--help)
      sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      DUMP="$arg"
      ;;
  esac
done

if [ "$FETCH" = 0 ] && [ ! -f "$DUMP" ]; then
  echo "덤프 없음: $DUMP" >&2
  echo "Slackdump export ZIP을 .github/scripts/slack.zip 에 두거나, 경로를 인자로 줘라." >&2
  echo "또는 --fetch 로 slack_secrets.txt에서 직접 긁어라." >&2
  exit 2
fi

# 추적 중인 파일에 미커밋 변경이 있으면 main 전환/rebase가 꼬인다. untracked(slack.zip 등)는 무시.
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "추적 파일에 커밋 안 된 변경이 있다. 먼저 커밋하거나 stash해라:" >&2
  git status --short --untracked-files=no >&2
  exit 2
fi

git fetch origin main
git switch main
git pull --ff-only origin main

if [ "$FETCH" = 1 ]; then
  FETCH_ARGS=(--out "$DUMP")
  if [ -n "${SLACK_SECRETS:-}" ]; then
    FETCH_ARGS+=(--secrets "$SLACK_SECRETS")
  fi
  python3 .github/scripts/slack_fetch.py "${FETCH_ARGS[@]}"
fi

if [ ! -f "$DUMP" ]; then
  echo "덤프 없음: $DUMP" >&2
  exit 2
fi
echo "덤프: $DUMP"

python3 .github/scripts/slack_stats.py --dump "$DUMP" --readme README.md

if git diff --quiet -- README.md; then
  echo "README 변경 없음 — 스냅샷이 이전과 같은가?"
  if [ "$FETCH" = 1 ]; then rm -f "$DUMP"; fi
  exit 0
fi

if [ "$PUSH" = 0 ]; then
  echo "README만 갱신함 (--no-push) — 커밋 안 함"
  exit 0
fi

git add README.md
BOT_NAME='github-actions[bot]'
BOT_EMAIL='41898282+github-actions[bot]@users.noreply.github.com'
GIT_AUTHOR_NAME="$BOT_NAME" GIT_AUTHOR_EMAIL="$BOT_EMAIL" \
GIT_COMMITTER_NAME="$BOT_NAME" GIT_COMMITTER_EMAIL="$BOT_EMAIL" \
  git commit -m "docs: Slack 활동 통계 갱신 [skip ci]"

for i in 1 2 3; do
  if git push origin main; then
    echo "push 완료: main"
    if [ "$FETCH" = 1 ]; then
      rm -f "$DUMP"
      echo "Slack 스냅샷 삭제: $DUMP (다음 실행 때 다시 긁는다)"
    else
      echo "덤프는 gitignore 처리됨: $DUMP — 필요 없으면 삭제해도 된다."
    fi
    exit 0
  fi
  echo "push 실패 — main 최신으로 rebase 후 재시도 ($i/3)"
  git pull --rebase origin main || {
    echo "rebase 실패 — 수동 확인 필요" >&2
    exit 1
  }
  sleep 3
done

echo "push 실패 — 수동 확인 필요" >&2
exit 1
