#!/usr/bin/env python3
"""호스트 VLM HTTP (표준 라이브러리만) — 컨테이너 vlm_task_planner가 호출.

  python3 server.py [--port 8000] [--host 127.0.0.1] [--config config.json]

  POST /plan {"command": "...", "objects": [{"id": 1, "class_name": "puck"}],
              "images": [{"data_b64": "<jpeg base64>"}],
              "replan": {"plan_id": 1, "failed_step": 2, "reason": "grasp_failed_all"},
              "history": [{"role": "user", "text": "..."}, {"role": "plan", "text": "..."}]}
    → {"ok": true, "goals": [{step, skill, target, relation, offset_m}]}
    → 못 짜면 {"ok": false, "error": "..."} (컨테이너는 템플릿으로 폴백)
  GET /health → {"ok": true, "backend": "..."}

  GET / → 사람 계획 페이지 (human.html) — backend "human"이면 /plan 요청이 여기 뜨고 사람이 단계를 짜서 보냄
  GET /human/pending → {"pending": [...], "history": [...]} (이미지 빼고) · GET /human/item?id=N (상세 — 물체·이미지·프롬프트)
  POST /human/submit {id, goals} (coordinator와 같은 검증) · POST /human/reject {id, error}

백엔드는 backends/: template(기본·오프라인) · claude_cli · opencode_cli · ollama(로컬) · openai_compat(외부사) · human(사람이 웹에서).
출력은 coordinator 검증(허용 동작·대상 id·step 순서)과 같은 규칙으로 검사한다.
"""

import argparse
import base64
import json
import sys
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "backends"))

from backends import template  # noqa: E402
from prompt import build as build_prompt  # noqa: E402

ALLOWED = {"navigate", "pick", "place", "inspect", "say"}
CONFIG = {}


def validate(goals, objects):
    ids = {str(o.get("id")) for o in objects if isinstance(o, dict)}
    if not isinstance(goals, list) or not goals:
        return "empty"
    for i, g in enumerate(goals, 1):
        if not isinstance(g, dict):
            return f"step {i}: 객체 아님"
        if g.get("step") != i:
            return f"step {i}: 순서 어긋남({g.get('step')})"
        if g.get("skill") not in ALLOWED:
            return f"step {i}: skill {g.get('skill')}"
        t = str(g.get("target", ""))
        if g.get("skill") in ("navigate", "pick", "place", "inspect") and t.isdigit() and t not in ids:
            return f"step {i}: target {t} not tracked"
        for im in ("images",):
            _ = im  # images는 검증 대상 아님 (용량 큰 base64 스킵)
    return ""


def check_images(images):
    total = 0
    for im in images[:2]:
        d = im.get("data_b64", "") if isinstance(im, dict) else ""
        if not d:
            continue
        try:
            raw = base64.b64decode(d, validate=True)
        except Exception:  # noqa: BLE001
            return "이미지 base64 오류"
        if isinstance(im, dict) and im.get("format", "jpeg") == "rgb8":
            w, h = im.get("width"), im.get("height")
            if not isinstance(w, int) or not isinstance(h, int):
                return "rgb8는 width·height(int) 필요"
            if len(raw) != w * h * 3:
                return "rgb8 크기 불일치"
        elif len(raw) > 500_000:
            return "이미지 500KB 초과 — 320px JPEG로 줄여라"
        total += len(raw)
    if total == 0 and images:
        return "이미지 비어 있음"
    return ""


def ai_conf():
    ai = CONFIG.get("ai") or {}
    return ai if isinstance(ai, dict) else {}


def system_text(ai):
    """시스템 프롬프트 = system_file(기본 system_prompt.md — 상대 경로는 이 폴더 기준) 내용 + system(덧붙일 말).
    요청마다 파일을 읽음 — 프롬프트를 고쳐도 서버를 다시 켤 필요 없음. 파일이 없으면 system만"""
    parts = []
    name = ai.get("system_file", "")
    if isinstance(name, str) and name.strip():
        p = Path(name)
        p = p if p.is_absolute() else HERE / p
        try:
            parts.append(p.read_text(encoding="utf-8").strip())
        except OSError:
            pass
    extra = ai.get("system", "")
    if isinstance(extra, str) and extra.strip():
        parts.append(extra.strip())
    return "\n\n".join(parts)


def dispatch(req, image_files=None):
    """image_files: begin_log이 저장한 PNG 절대경로 — opencode_cli만 --file로 쓴다."""
    ai = ai_conf()
    backend = ai.get("backend", "template")
    system = system_text(ai)
    if backend == "claude_cli":
        from backends import claude_cli
        return claude_cli.plan(req, model=ai.get("model", "default"),
                               timeout_s=ai.get("timeout_s", 60), system=system,
                               image_files=image_files)
    if backend == "opencode_cli":
        from backends import opencode_cli
        return opencode_cli.plan(req, model=ai.get("model", "default"),
                                 timeout_s=ai.get("timeout_s", 60), system=system,
                                 image_files=image_files)
    if backend == "ollama":
        from backends import ollama
        return ollama.plan(req, model=ai.get("model"),
                           endpoint=ai.get("endpoint"),
                           timeout_s=ai.get("timeout_s"), system=system)
    if backend == "human":
        from backends import human
        from images import prepare
        return human.plan(req, timeout_s=ai.get("timeout_s", 600), images=prepare(req),
                          prompt=build_prompt(req, system))
    if backend == "openai_compat":
        from backends import openai_compat
        return openai_compat.plan(req, endpoint=ai.get("endpoint", ""),
                                  model=ai.get("model", "default"),
                                  timeout_s=ai.get("timeout_s", 60), system=system)
    return template.plan(req)


def begin_log(req):
    """로그 디렉토리 생성 + prepare(req) JPEG(원본·마킹) → PNG 저장.
    → (dir, [{file, bytes, path}]) — path는 모델에 --file로 넘기는 절대경로."""
    import uuid

    from images import prepare, to_png

    d = HERE / "log" / f"plan-{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:6]}"
    d.mkdir(parents=True, exist_ok=True)
    saved = []
    for i, im in enumerate(prepare(req)):
        try:
            blob = base64.b64decode(im.get("data_b64", ""), validate=True)
        except Exception:  # noqa: BLE001
            blob = b""
        png = to_png(blob, "jpeg", None, None) if blob else None
        if png:
            (d / f"request-image{i + 1}.png").write_bytes(png)
            saved.append({"file": f"request-image{i + 1}.png", "bytes": len(png),
                          "path": str((d / f"request-image{i + 1}.png").resolve())})
        else:
            saved.append({"file": None, "bytes": 0, "path": ""})
    return d, saved


def log_plan(req, code, resp, t_arrive=None, raw="", t_dispatch=None, pre=None):
    """요청 건당 log/plan-YYYYMMDD-HHMMSS-<rand>/ 에 ai.json·request.json·response.json·request-imageN.png.
    ai.json은 AI 관련만: {backend, model, endpoint, timeout_s,
               request: {time, elapsed_s(도착→백엔드 호출), prompt},
               response: {time, elapsed_s(호출→완료), raw(LLM 날출력 문자열 그대로)}}.
    request.json(원문+이미지 파일명)·response.json{code, resp}은 그대로 별도 보관. 이미지 원문은 PNG 파일로만.
    pre=(dir, saved): do_POST가 dispatch 전에 미리 저장했으면 재사용 (검증 실패 로그는 여기서 생성)."""
    try:
        if pre is not None:
            d, saved = pre
        else:
            d, saved = begin_log(req)
        now = time.time()
        arrive = t_arrive if isinstance(t_arrive, (int, float)) else now
        call = t_dispatch if isinstance(t_dispatch, (int, float)) else arrive
        ai = ai_conf()
        try:
            prompt_text = build_prompt(req if isinstance(req, dict) else {}, system_text(ai))
        except Exception:  # noqa: BLE001
            prompt_text = ""
        suffix = ""
        if ai.get("backend") == "opencode_cli":
            suffix = "\n도구 사용 없이 JSON으로만 답하라."  # backends/opencode_cli.py와 동일 유지
        req_out = dict(req) if isinstance(req, dict) else {}
        req_out["images"] = [
            {"file": s["file"], "bytes": s["bytes"]} for s in saved
        ]
        (d / "request.json").write_text(json.dumps(req_out, ensure_ascii=False, indent=2))
        (d / "response.json").write_text(json.dumps({"code": code, "resp": resp},
                                                     ensure_ascii=False, indent=2))
        (d / "ai.json").write_text(json.dumps({
            "backend": ai.get("backend", "template"),
            "model": ai.get("model", "default"),
            "endpoint": ai.get("endpoint", ""),
            "timeout_s": ai.get("timeout_s", 60),
            "request": {
                "time": datetime.fromtimestamp(arrive).isoformat(timespec="seconds"),
                "elapsed_s": round(call - arrive, 2),
                "prompt": prompt_text + suffix,
            },
            "response": {
                "time": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
                "elapsed_s": round(now - call, 2),
                "raw": (raw[:20000] + "…[truncated]" if len(raw) > 20000 else raw)
                       if isinstance(raw, str) else "",
            },
        }, ensure_ascii=False, indent=2))
    except Exception:  # noqa: BLE001 — 로깅 실패가 서빙을 막지 않음
        pass


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n) or b"{}") if n <= 4_000_000 else None
        except (ValueError, json.JSONDecodeError):
            return None

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/health":
            self._send(200, {"ok": True, "backend": ai_conf().get("backend", "template")})
        elif path in ("/", "/human"):  # 사람 계획 페이지
            body = (HERE / "human.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/human/item":
            from urllib.parse import parse_qs, urlparse
            from backends import human
            try:
                rid = int(parse_qs(urlparse(self.path).query).get("id", ["0"])[0])
            except ValueError:
                rid = 0
            item = human.detail(rid)
            self._send(200 if item else 404, {"ok": True, "item": item} if item else {"ok": False, "error": "없음"})
        elif path == "/human/pending":
            from backends import human
            self._send(200, {"ok": True, "backend": ai_conf().get("backend", "template"),
                             "allowed": sorted(ALLOWED), "pending": human.pending(), "history": human.history()})
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def human_answer(self, reject):
        """POST /human/submit {id, goals} (계획 검증 — coordinator와 같은 규칙) · /human/reject {id, error}"""
        from backends import human
        body = self._body()
        if not isinstance(body, dict):
            self._send(400, {"ok": False, "error": "JSON 형식 오류"})
            return
        item = human.get(body.get("id"))
        if item is None:
            self._send(404, {"ok": False, "error": "이미 끝난 요청 (답했거나 시간 초과)"})
            return
        if reject:
            ok = human.answer(item["id"], error=str(body.get("error") or "사람이 거절"))
        else:
            goals = body.get("goals")
            why = validate(goals, item["req"].get("objects", []))
            if why:
                self._send(400, {"ok": False, "error": f"검증 실패: {why}"})
                return
            ok = human.answer(item["id"], goals=goals)
        self._send(200 if ok else 409, {"ok": ok} if ok else {"ok": False, "error": "이미 끝난 요청"})

    def do_POST(self):
        if self.path in ("/human/submit", "/human/reject"):
            self.human_answer(self.path.endswith("reject"))
            return
        if self.path != "/plan":
            self._send(404, {"ok": False, "error": "not found"})
            return
        req = {}
        t_arrive = time.time()
        stamp = {"dispatch": None, "pre": None}

        def done(code, resp, raw=""):
            log_plan(req, code, resp, t_arrive, raw, stamp["dispatch"], stamp["pre"])
            self._send(code, resp)

        try:
            n = int(self.headers.get("Content-Length", 0))
        except ValueError:
            n = 0
        if n > 4_000_000:
            done(413, {"ok": False, "error": "요청 4MB 초과"})
            return
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            done(400, {"ok": False, "error": "JSON 형식 오류"})
            return
        objects = req.get("objects", [])
        if not isinstance(objects, list):
            done(400, {"ok": False, "error": "objects 배열 필요"})
            return
        if "history" in req and not isinstance(req.get("history"), list):
            done(400, {"ok": False, "error": "history 배열 필요"})
            return
        images = req.get("images", [])
        if images:
            err = check_images(images)
            if err:
                done(400, {"ok": False, "error": err})
                return
        stamp["pre"] = begin_log(req)  # PNG 저장 + 로그 dir 확보 → --file은 이 경로 사용
        image_files = [s["path"] for s in stamp["pre"][1] if s["path"]]
        stamp["dispatch"] = time.time()
        goals, err, raw = dispatch(req, image_files)
        if err or goals is None:
            done(200, {"ok": False, "error": err or "계획 실패"}, raw)
            return
        why = validate(goals, objects)
        if why:
            done(200, {"ok": False, "error": f"검증 실패: {why}"}, raw)
            return
        done(200, {"ok": True, "goals": goals}, raw)

    def log_message(self, fmt, *args):  # 시끄러운 기본 로그 억제
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=None)  # 주면 config api.port 덮어씀
    ap.add_argument("--host", default=None)  # 주면 config api.host 덮어씀
    ap.add_argument("--config", default=str(HERE / "config.json"))
    args = ap.parse_args()
    try:
        CONFIG.update(json.loads(Path(args.config).read_text()))
    except (OSError, json.JSONDecodeError) as e:
        print(f"config 읽기 실패({args.config}): {e} — template 기본값", flush=True)
        CONFIG.update({"backend": "template"})
    api = CONFIG.get("http") or {}
    if not isinstance(api, dict):
        api = {}
    try:
        port = int(api.get("port", 8000))
    except (TypeError, ValueError):
        port = 8000
    host = api.get("host") or "127.0.0.1"
    if args.port is not None:
        port = args.port
    if args.host is not None:
        host = args.host
    srv = ThreadingHTTPServer((host, port), Handler)
    print(f"ai-service {host}:{port} backend={ai_conf().get('backend')} (GET /health, POST /plan, 사람 계획 페이지 http://{host}:{port}/)",
          flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
