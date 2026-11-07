"""사람이 계획 — POST /plan 요청을 웹 페이지(GET /, human.html)에 띄우고 사람이 낸 계획을 그대로 돌려준다.

  config "backend": "human", "timeout_s": 600 (사람이 답할 시간 — 컨테이너 plan.yaml vlm_timeout_s도 그 이상으로)
  요청 하나 = 대기 항목 하나. /plan 처리 스레드는 사람이 답하거나(answer) timeout_s가 지날 때까지 기다린다.
  ThreadingHTTPServer라 기다리는 동안에도 웹 페이지 요청(GET /human/pending · POST /human/submit)은 따로 처리된다.
"""

import itertools
import threading
import time

_lock = threading.Lock()
_pending = {}          # id → 대기 항목
_answered = []         # 최근 답한 것 (페이지 기록용, HISTORY_N개)
_ids = itertools.count(1)
HISTORY_N = 20


def plan(req, timeout_s=600, images=None, prompt=""):
    """→ (goals, err, raw). images = images.prepare(req) 결과(원본·마킹) — 페이지가 그대로 보여 줌"""
    item = {"id": next(_ids), "req": req, "images": images or [], "prompt": prompt,
            "created": time.time(), "timeout_s": float(timeout_s), "event": threading.Event(), "result": None}
    with _lock:
        _pending[item["id"]] = item
    answered = item["event"].wait(float(timeout_s))
    with _lock:
        _pending.pop(item["id"], None)
        if not answered:
            item["result"] = (None, f"사람 응답 없음 — 시간 초과 ({timeout_s:g}초)", "")
        _answered.insert(0, {"id": item["id"], "command": req.get("command", ""), "created": item["created"],
                             "finished": time.time(), "goals": item["result"][0], "error": item["result"][1]})
        del _answered[HISTORY_N:]
    return item["result"]


def _public(i, full):
    now = time.time()
    req = i["req"]
    out = {"id": i["id"], "command": req.get("command", ""), "replan": req.get("replan") or None,
           "n_objects": len(req.get("objects", [])), "timeout_s": i["timeout_s"],
           "age_s": round(now - i["created"], 1), "left_s": round(i["timeout_s"] - (now - i["created"]), 1)}
    if full:  # 상세 — 요청 원문(이미지 base64 빼고) · 원본·마킹 이미지 · AI가 받았을 프롬프트
        out.update(req={k: v for k, v in req.items() if k != "images"}, images=i["images"], prompt=i["prompt"])
    return out


def pending():
    """페이지용 대기 목록 (오래된 것부터, 1초마다 불림 — 이미지는 빼고)"""
    with _lock:
        return [_public(i, False) for i in sorted(_pending.values(), key=lambda i: i["id"])]


def detail(rid):
    with _lock:
        i = _pending.get(rid)
        return _public(i, True) if i else None


def history():
    with _lock:
        return list(_answered)


def get(rid):
    with _lock:
        return _pending.get(rid)


def answer(rid, goals=None, error=None):
    """사람의 답 — goals(검증은 부르는 쪽) 또는 error(거절 사유 — 컨테이너는 say로 보여 줌). 이미 끝났으면 False"""
    with _lock:
        item = _pending.get(rid)
        if item is None or item["event"].is_set():
            return False
        raw = "human: " + (error if error else f"{len(goals or [])} steps")
        item["result"] = (None, error, raw) if error else (goals, None, raw)
        item["event"].set()
        return True
