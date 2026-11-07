"""OpenAI 호환 HTTP 백엔드 (표준 라이브러리만) — 외부 회사 API·로컬 ollama용.

POST {endpoint}/chat/completions {"model": ..., "messages": [{"role": "user",
"content": [{"type": "text", "text": <프롬프트>},
{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}} ...]}]},
temperature=0, response_format=json_object.
키는 환경변수(OPENAI_API_KEY 등)에서 읽고 이미지에 절대 굽지 않는다.
단 localhost endpoints(ollama 등 로컬)는 키 없이 호출한다.
"""

import json
import os
import urllib.request
from urllib.parse import urlparse

from backends import opencode_cli
from images import prepare
from prompt import build


def _is_local(endpoint):
    try:
        host = urlparse(endpoint).hostname or ""
    except ValueError:
        return False
    return host in ("localhost", "127.0.0.1", "::1")


def plan(req, endpoint="https://api.openai.com/v1", model="gpt-4o-mini", timeout_s=60, system="", options=None):
    """→ (goals, err, raw) — raw는 모델 날출력 문자열 그대로 (파싱 실패해도 버리지 않음)."""
    key = os.environ.get("OPENAI_API_KEY", "")
    if not key and not _is_local(endpoint):
        return None, "OPENAI_API_KEY 없음", ""
    content = [{"type": "text", "text": build(req)}]
    for im in prepare(req):  # 원본·마킹 최대 4장
        content.append({"type": "image_url",
                        "image_url": {"url": "data:image/jpeg;base64," + im["data_b64"]}})
    messages = []
    if isinstance(system, str) and system.strip():
        messages.append({"role": "system", "content": system.strip()})
    messages.append({"role": "user", "content": content})
    body = {
        "model": model,
        "temperature": 0,
        "response_format": {"type": "json_object"},
        "messages": messages,
    }
    if isinstance(options, dict) and options:
        body["options"] = options  # ollama 전용 (num_ctx 등) — 외부 API는 무시됨
    body = json.dumps(body).encode()
    headers = {"Content-Type": "application/json"}
    if key:
        headers["Authorization"] = "Bearer " + key
    r = urllib.request.Request(endpoint.rstrip("/") + "/chat/completions", data=body,
                               headers=headers)
    try:
        with urllib.request.urlopen(r, timeout=timeout_s) as res:
            out = json.loads(res.read())
    except Exception as e:  # noqa: BLE001 — 네트워크 실패는 문자열로 전달
        return None, f"http 실패: {e}"[:200], ""
    try:
        text = out["choices"][0]["message"]["content"]
    except (KeyError, IndexError):
        return None, "응답 goals 파싱 실패", ""
    raw = text if isinstance(text, str) else ""
    # qwen3-vl 등은 설명문+JSON을 뱉는다 — opencode 쪽 스캐너로 배열 회수
    goals, err = opencode_cli._goals_from_text(raw)
    if err or not goals:
        return None, err or "응답에 goals 없음", raw
    return goals, None, raw
