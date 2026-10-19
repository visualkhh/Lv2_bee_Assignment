"""opencode CLI 백엔드 — `opencode run --format json [--model m] [--file img.jpg …] "<프롬프트>"` 호출.

이미지는 base64 → 임시 JPEG로 내려 `--file`로 첨부한다 (서버가 장당 500KB 제한済).
`opencode` 없으면 에러 반환 (컨테이너는 템플릿 폴백).
"""

import base64
import json
import shutil
import subprocess
import tempfile

from images import prepare
from prompt import build


def _extract_goals(obj):
    if isinstance(obj, list):
        return obj or None
    if isinstance(obj, dict):
        if isinstance(obj.get("goals"), list):
            return obj["goals"] or None
        for key in ("response", "text", "content", "message", "output", "result", "data"):
            v = obj.get(key)
            if isinstance(v, (list, dict)):
                found = _extract_goals(v)
                if found:
                    return found
            elif isinstance(v, str):
                try:
                    found = _extract_goals(json.loads(v))
                except (json.JSONDecodeError, ValueError):
                    continue
                if found:
                    return found
    return None


def _walk_strings(obj, buf):
    if isinstance(obj, str):
        buf.append(obj)
    elif isinstance(obj, list):
        for v in obj:
            _walk_strings(v, buf)
    elif isinstance(obj, dict):
        for v in obj.values():
            _walk_strings(v, buf)


def _scan_arrays(s):
    dec, best = json.JSONDecoder(), None  # JSON 감싼 설명문에서 배열 스캔
    for i, ch in enumerate(s):
        if ch != "[":
            continue
        try:
            goals = _extract_goals(dec.raw_decode(s[i:])[0])
        except (json.JSONDecodeError, ValueError):
            continue
        if goals and (best is None or len(goals) > len(best)):
            best = goals
    return best


def _goals_from_text(text):
    t = (text or "").strip()
    if not t:
        return None, "빈 출력"
    try:
        goals = _extract_goals(json.loads(t))
        if goals:
            return goals, None
    except (json.JSONDecodeError, ValueError):
        pass
    if _scan_arrays(t):  # 단일 JSON이 아니어도 통째 스캔 한 번
        return _scan_arrays(t), None
    buf = []  # opencode --format json은 JSONL 스트림: 줄별 파싱 + 텍스트 모아 스캔
    for line in t.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except (json.JSONDecodeError, ValueError):
            buf.append(line)
            continue
        goals = _extract_goals(obj)
        if goals:
            return goals, None
        _walk_strings(obj, buf)
    best = _scan_arrays("\n".join(buf))
    if best:
        return best, None
    return None, "응답에 goals 없음"


def plan(req, model="default", timeout_s=60, system="", image_files=None):
    """→ (goals, err, raw) — raw는 CLI 날출력 문자열 그대로 (파싱 실패해도 버리지 않음).
    image_files가 있으면 로그 PNG 경로를 --file로 그대로 쓰고(stdin으로 프롬프트 전달),
    없으면 base64를 임시 JPEG로 내려 --file로 첨부한다."""
    if shutil.which("opencode") is None:
        return None, "opencode CLI 없음", ""
    prompt = build(req, system) + "\n도구 사용 없이 JSON으로만 답하라."
    cmd = ["opencode", "run", "--format", "json"]
    if model and model != "default":
        cmd += ["--model", model]
    tmp = []
    given = [f for f in (image_files or [])]
    try:
        if given:
            for f in given:
                cmd += ["--file", f]
        else:
            for im in prepare(req):  # 원본·마킹 최대 4장
                try:
                    raw = base64.b64decode(im["data_b64"], validate=True)
                except Exception:  # noqa: BLE001
                    return None, "이미지 base64 오류", ""
                f = tempfile.NamedTemporaryFile(delete=False, suffix=".jpg",
                                                prefix=f"{im['camera']}-{im['kind']}-")
                f.write(raw)
                f.close()
                tmp.append(f.name)
                cmd += ["--file", f.name]
        try:
            p = subprocess.run(cmd, input=prompt, capture_output=True, text=True,
                               timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None, "opencode timeout", ""
        if p.returncode != 0:
            return None, f"opencode exit={p.returncode}: {p.stderr[:200]}", p.stdout or ""
        goals, err = _goals_from_text(p.stdout)
        return goals, err, p.stdout
    finally:
        import os

        for path in tmp:
            try:
                os.unlink(path)
            except OSError:
                pass
