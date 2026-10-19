"""VLM 프롬프트 조립 — 명령 + 물체 + 이미지 수 + 재계획 사유 + 대화 기록 → 텍스트 한 덩이.

컨테이너가 보낸 JSON을 사람·모델이 읽는 지시문으로 바꾼다.
서버는 무상태: history는 요청에 실려 오고 저장하지 않는다 (최근 20개·개당 500자까지만).
출력 스키마는 coordinator 검증(03-plan.md:67)과 동일하게 고정한다.
"""

import math

ALLOWED = "navigate · pick · place · say"
# 적재함 (coordinator target "tray") — MISSION ③ MOVE_TO_B는 손에 들고 옮기면 인정 안 됨
TRAY_RULE = ("적재함: pick·place의 target에 \"tray\"를 쓰면 로봇 뒤 적재함이다 — place tray = 손에 든 퍽을 다음 빈 칸에 싣기, "
             "pick tray = 실은 퍽을 하나 꺼내기(나중에 실은 것부터). 최대 4개. MOVE_TO_B는 퍽마다 navigate·pick·place tray로 싣고 "
             "B 지역으로 navigate한 뒤 실은 수만큼 pick tray·place <B 지역 id>로 내려놓는다 (손에 들고 옮기지 말 것). "
             "같은 곳에 여러 번 place하면 coordinator가 겹치지 않게 비켜 놓는다.")
# pick·place는 팔이 닿아야 함 — 바닥 퍽은 로봇 중심에서 약 0.13~0.31m (simulator 팔 범위 · coordinator reach_m 0.30)
REACH_M = 0.30
REACH_RULE = (f"팔이 닿는 곳: 로봇 중심에서 약 {REACH_M:.2f}m 안. pick·place <물체 id>의 대상이 그보다 멀면 바로 앞 단계에 "
              "navigate <같은 id> (relation front, offset_m 0.1)를 넣어라 — 대상까지 거리(dist)는 물체 목록에 있다. "
              "색·종류를 말한 명령은 color·class와 마킹 이미지(번호=id)로 대상을 고른다.")
HISTORY_N = 20
HISTORY_CHARS = 500


def build(req, system=""):
    lines = []
    if isinstance(system, str) and system.strip():
        lines.append(f"시스템: {system.strip()}")
    lines.append(f"명령: {req.get('command', '')}")
    objs = req.get("objects", [])
    robot = req.get("robot")
    rxy = None
    if isinstance(robot, dict) and all(isinstance(robot.get(k), (int, float)) for k in ("x", "y")):
        rxy = (float(robot["x"]), float(robot["y"]))
        yaw = robot.get("yaw")
        heading = f" 바라보는 방향 {math.degrees(yaw):+.0f}°" if isinstance(yaw, (int, float)) else ""
        lines.append(f"로봇 위치(map): ({rxy[0]:.2f},{rxy[1]:.2f}){heading}")
    lines.append(f"보이는 물체({len(objs)}개, 좌표=map 기준 m):")
    for o in objs:
        if isinstance(o, dict):
            pos = o.get("position", [])
            size = o.get("size", [])
            extra = ""
            if len(pos) == 3:
                extra += f" pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
            if len(size) == 3:
                extra += f" size=({size[0]:.2f},{size[1]:.2f},{size[2]:.2f})"
            if rxy is not None and len(pos) == 3:
                extra += f" dist={math.hypot(pos[0] - rxy[0], pos[1] - rxy[1]):.2f}m"
            if isinstance(o.get("color"), str) and o["color"]:
                extra += f" color={o['color']}"
            if isinstance(o.get("confidence"), (int, float)):
                extra += f" conf={o['confidence']:.2f}"
            px = o.get("px", {})
            if isinstance(px, dict):
                for cam in ("pole_cam", "gripper_cam"):
                    if cam in px and isinstance(px[cam], (list, tuple)) and len(px[cam]) == 2:
                        extra += f" {cam.split('_')[0]}px=({px[cam][0]},{px[cam][1]})"
            lines.append(f"  - id={o.get('id')} class={o.get('class_name')}{extra}")
    n_img = len(req.get("images", []))
    lines.append(f"첨부 이미지: {n_img}장 — 카메라별 원본·마킹(번호=id) 순으로 전달 "
                 "(예: pole 원본·pole 마킹·gripper 원본·gripper 마킹, 마킹 없는 카메라는 원본만)")
    rp = req.get("replan") or {}
    if rp:
        lines.append(f"재계획: plan={rp.get('plan_id')} step={rp.get('failed_step')} 사유={rp.get('reason')}")
        lines.append("같은 실패를 반복하지 말고 접근 방향·순서를 바꿔라.")
    hist = req.get("history", [])
    if isinstance(hist, list) and hist:
        lines.append(f"이전 대화(최근 {HISTORY_N}개까지):")
        for h in hist[-HISTORY_N:]:
            if isinstance(h, dict):
                text = str(h.get("text", ""))[:HISTORY_CHARS]
                lines.append(f"  - {h.get('role', '?')}: {text}")
    lines.append(REACH_RULE)
    lines.append(TRAY_RULE)
    lines.append(f"허용 동작: {ALLOWED}. JSON 배열로만 답하라: "
                 '[{"step": 1, "skill": "...", "target": "<물체id|tray|말할내용>", "relation": "front|on|", "offset_m": 0.0}]')
    return "\n".join(lines)
