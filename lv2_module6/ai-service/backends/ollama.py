"""로컬 ollama 백엔드 — 네이티브 `/api/chat` 직접 호출 (stdlib urllib).

`/v1/chat/completions` 호환 주소는 `options`(num_ctx)를 무시해서 262k 컨텍스트로
뜨고, thinking 모델은 reasoning 폭증으로 타임아웃난다. 네이티브 주소로
`options.num_ctx=8192`·`think:false`·`format:json`을 고정한다.
ollama가 꺼져 있거나 모델이 없으면 에러 반환 (컨테이너는 템플릿 폴백).
기본값은 여기 DEFAULTS 한 곳에만 — server.py는 config.json 값을 그대로 넘긴다.
"""

import json
import urllib.request

from backends import opencode_cli
from images import prepare
from prompt import build

DEFAULTS = {
    "model": "llama4",
    "endpoint": "http://localhost:11434",
    "timeout_s": 120,
}

# 로컬 VLM 프롬프트는 1~2천 토큰이면 충분 — 262k 기본값으로 뜨면
# prefill·KV 캐시가 폭증해 매 요청이 타임아웃난다. 작게 고정.
NUM_CTX = 8192


def _root(endpoint):
    root = (endpoint or DEFAULTS["endpoint"]).rstrip("/")
    if root.endswith("/v1"):
        root = root[:-3]  # /v1/chat/completions 호환주소 → 네이티브 루트
    return root


def plan(req, model=None, endpoint=None, timeout_s=None, system=""):
    """→ (goals, err, raw) — raw는 모델 날출력 문자열 그대로."""
    model = model or DEFAULTS["model"]
    timeout_s = timeout_s or DEFAULTS["timeout_s"]
    user = {"role": "user", "content": build(req, system)}
    imgs = [im["data_b64"] for im in prepare(req) if im.get("data_b64")]
    if imgs:
        user["images"] = imgs  # 네이티브는 data-URI가 아니라 base64 리스트
    body = json.dumps({
        "model": model,
        "messages": [user],
        "stream": False,
        "format": "json",
        "think": False,
        "options": {"num_ctx": NUM_CTX},
        "keep_alive": "10m",
    }).encode()
    r = urllib.request.Request(_root(endpoint) + "/api/chat", data=body,
                               headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(r, timeout=timeout_s) as res:
            out = json.loads(res.read())
    except Exception as e:  # noqa: BLE001 — 네트워크 실패는 문자열로 전달
        return None, f"http 실패: {e}"[:200], ""
    try:
        text = out["message"]["content"]
    except (KeyError, TypeError):
        return None, "응답 goals 파싱 실패", ""
    raw = text if isinstance(text, str) else ""
    goals, err = opencode_cli._goals_from_text(raw)
    if err or not goals:
        return None, err or "응답에 goals 없음", raw
    return goals, None, raw
