"""Claude CLI 백엔드 — `claude -p "<프롬프트>" --output-format json` 호출.

이미지는 로그 PNG 절대경로를 프롬프트에 적어 Read 도구로 직접 열게 한다
(--file은 내려받기용이라 로컬 첨부에 못 씀).
`claude` 없으면 에러 반환 (컨테이너는 템플릿 폴백).
"""

import json
import shutil
import subprocess

from backends import opencode_cli
from prompt import build


def plan(req, model="default", timeout_s=60, system="", image_files=None):
    """→ (goals, err, raw) — raw는 CLI 날출력 문자열 그대로 (파싱 실패해도 버리지 않음)."""
    if shutil.which("claude") is None:
        return None, "claude CLI 없음", ""
    prompt = build(req, system)
    files = [f for f in (image_files or [])]
    if files:
        prompt += ("\n첨부 이미지 — Read 도구로 직접 열어서 볼 것 (설명만 보고 추측 금지):\n" +
                   "\n".join(f"- {f}" for f in files))
    try:
        p = subprocess.run(["claude", "-p", prompt, "--allowedTools", "Read",
                            "--output-format", "json"],
                           capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None, "claude timeout", ""
    if p.returncode != 0:
        return None, f"claude exit={p.returncode}: {p.stderr[:200]}", p.stdout or ""
    try:
        out = json.loads(p.stdout)
    except json.JSONDecodeError:
        return None, "claude 출력 JSON 파싱 실패", p.stdout
    text = out.get("result", "") if isinstance(out, dict) else ""
    if isinstance(text, list):
        text = json.dumps(text)
    goals, err = opencode_cli._goals_from_text(text if isinstance(text, str) else "")
    if err or not goals:
        return None, err or "claude 응답에 goals 없음", p.stdout
    return goals, None, p.stdout
