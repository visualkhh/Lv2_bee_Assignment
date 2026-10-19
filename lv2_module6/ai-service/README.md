# 호스트 VLM HTTP (`ai-service/`)

컨테이너 `vlm_task_planner`가 호출하는 호스트 서버. 키·모델·CLI는 여기, 컨테이너는 호출만.
```shell
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회
```
```bash
./test.sh                                                        # 34개, 외부 호출 없음 (CLI·HTTP 전부 스텁)
./server.sh [--port 8000] [--host 127.0.0.1] [--config config.json]
```

## 테스트

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회 (표준 라이브러리만이라 설치할 건 없음)
./test.sh                                                            # 36개, 외부 호출 없음 (CLI·HTTP 전부 스텁)
```

* `tests/test_prompt.py` — 프롬프트 조립·history 캡·실물 이미지 동봉 (`tests/data/`: 시뮬레이터 헤드리스 렌더 실장면 — pole_cam 미로+퍽 · gripper_cam 근접)
* `tests/test_server.py` — goals 검증·이미지 검사·디스패치
* `tests/test_backends.py` — template·opencode 파서(JSONL 포함)·ollama 위임
* 진짜 모델 호출은 테스트에 없음. 수동 확인은 서버 띄우고 `POST /plan` (한도·과금 주의)
* 로그: 요청 건당 `log/plan-날짜-시간-rand/` 에 `request.json`(원문)·`response.json`(`{code, resp}`)·`ai.json`(`{backend, model, endpoint, timeout_s, request:{time, elapsed_s, prompt}, response:{time, elapsed_s, raw}}` AI 송수신)·`request-image1.png`·`request-image2.png` (gitignore, 이미지 원문은 PNG 파일로만)

기본값은 `config.json`(opencode 내용). 대안은 `config/`에 백엔드별로 (`template`·`claude`·`ollama`·`opencode`) — `--config`로 고르면 된다.

* `POST /plan` → `{"ok": true, "goals": [...]}` 또는 `{"ok": false, "error"}` (후자면 컨테이너는 템플릿 폴백)
* 요청: `command`·`objects[{id, class_name, position(map), size, confidence, color, camera, px{카메라: [u, v]}}]`(컨테이너 vlm_task_planner가 `/tracked_objects`에서 — px는 그 카메라 영상 속 박스 중심, 이미지 마킹에 씀)·`robot{x, y, yaw}`(map, 로봇 위치 — 프롬프트에 물체마다 거리 dist를 붙여 팔이 안 닿으면 navigate를 먼저 넣게)·`images[{data_b64}]`(320px JPEG, 장당 500KB 제한)·`replan{plan_id, failed_step, reason}`·`history[{role, text}]`(선택, 최근 20개·개당 500자까지 프롬프트에 포함 — 서버는 저장 안 함)
* 출력 검증은 coordinator와 동일(허용 동작·대상 id·step 순서) — 실패면 `ok:false`
* 백엔드는 `backends/`: `template`(기본·오프라인) · `claude_cli` · `opencode_cli`(`opencode run --format json`, `--model`·`--file` 첨부) · `ollama`(로컬, 키 불필요) · `openai_compat`(외부사, `OPENAI_API_KEY`는 환경변수, localhost는 키 불필요) — `config.json`의 `backend`로 전환, 계약은 유지.

## 설정 (`config/`)

`python3 server.py [--port 8000] [--host 127.0.0.1] [--config config.json]` — 파일이 없거나 깨지면 `template` 기본값으로 뜬다. 모르는 `backend` 값도 `template`으로 동작한다. `--port`·`--host`를 주면 config의 `http`를 덮어쓴다.

| 키 | 내용 |
| --- | --- |
| `http` | 서버 바인딩 `{host, port}`. CLI `--port`·`--host`가 있으면 덮어씀. 파일별 기본 포트: opencode 8000 · ollama 8001 · claude 8002 · template 8003 (동시 기동 가능) |
| `ai.backend` | `"template"`(고정 4단계·오프라인) · `"claude_cli"` · `"opencode_cli"` · `"ollama"`(로컬) · `"openai_compat"`(외부 OpenAI 호환 API) |
| `ai.model` | CLI·API에 넘길 모델명. `"default"`면 각자 기본값 사용 |
| `ai.endpoint` | `openai_compat`·`ollama` 전용 — 뒤에 `/chat/completions`가 붙는다. CLI·template 백엔드는 안 씀 (config에 없어도 됨) |
| `ai.timeout_s` | 백엔드 호출 제한 [초] (기본 60) — 넘기면 `{"ok": false, "error"}` 반환, 컨테이너는 템플릿 폴백 |
| `ai.system_file` | 시스템 프롬프트 파일 (모든 설정 기본 **`system_prompt.md`** — 역할 · 미션 · 로봇 · 입력 읽는 법 · 허용 동작 · 규칙 · 재계획 사유별 대응 · 출력 형식). 요청마다 읽어서 고쳐도 서버 재시작 불필요, 상대 경로는 이 폴더 기준 |
| `ai.system` | 덧붙일 말 (기본 `""`) — `system_file` 내용 뒤에 붙음. 합친 것이 맨 앞에 `시스템: …`으로 들어감. `openai_compat`·`ollama`는 `system` 역할 메시지로, CLI 백엔드는 텍스트 앞에 붙여서 전송 |

백엔드별 `model` 해석:

* `template` — `model`·`endpoint` 무시 (VLM 없이 고정 계획)
* `claude_cli` — 현재 `model` 미사용(예약). `claude` CLI가 PATH에 있어야 함
* `opencode_cli` — `"default"`면 `--model` 생략, 그 외는 그대로 전달 (형식 `provider/model#variant`). 이미지는 임시 JPEG로 `--file` 첨부 후 삭제
* `openai_compat` — `model`을 요청에 그대로 넣음. 외부 API는 `endpoint`+키 필요. 키는 파일이 아닌 환경변수 `OPENAI_API_KEY`에서만 읽음 (localhost endpoints는 키 없이 호출)
* `ollama` — `openai_compat`에 위임하는 로컬 전용. 값은 `config/` 파일의 `ai`에서 읽고, 키가 빠지면 `backends/ollama.py` DEFAULTS(`llama4`·`localhost:11434`·120초)

```json
{ "http": {"host": "127.0.0.1", "port": 8000},
  "ai": {"backend": "opencode_cli", "model": "default", "endpoint": "https://api.openai.com/v1", "timeout_s": 60} }
```

## 예시 — claude · opencode · ollama

`config/`에 파일로 있다. `--config`로 고르면 된다 (`server.py` 재시작 필요).

```json
// config/claude.json (PATH에 `claude` 필요, model은 현재 미사용·예약)
{ "http": {"host": "127.0.0.1", "port": 8002},
  "ai": {"backend": "claude_cli", "model": "default", "timeout_s": 60} }
```

```json
// config.json — 지금 기본값 (PATH에 `opencode` 필요, model 형식 provider/model#variant)
{ "http": {"host": "127.0.0.1", "port": 8000},
  "ai": {"backend": "opencode_cli", "model": "opencode-go/muse-spark-1.3-contributor", "endpoint": "https://api.openai.com/v1", "timeout_s": 120} }
```

```json
// config/ollama.json (`ollama pull llama4` 먼저, `ollama list`의 이름과 model 일치시킬 것)
{ "http": {"host": "127.0.0.1", "port": 8001},
  "ai": {"backend": "ollama", "model": "llama4", "endpoint": "http://localhost:11434/v1", "timeout_s": 120} }
```