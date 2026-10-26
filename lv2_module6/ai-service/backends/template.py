"""고정 템플릿 — vlm_task_planner.cpp make_plan()과 동일.

[navigate(퍽 앞) → pick(퍽) → navigate(장소 태그) → place(장소 태그)]
이미지·재계획 문맥은 무시한다 (오프라인 기본값·CI용).
"""


def find(objects, cls):
    for o in objects:
        if isinstance(o, dict) and o.get("class_name") == cls:
            return str(o.get("id", ""))
    return ""


def plan(req):
    """→ (goals, err, raw) — 템플릿은 LLM을 안 쓰므로 raw는 항상 ""."""
    objects = req.get("objects", [])
    puck = find(objects, "puck")
    goal = find(objects, "location")
    if not puck or not goal:
        return None, "puck·location 없음 — 계획 보류", ""
    return [
        {"step": 1, "skill": "navigate", "target": puck, "relation": "front", "offset_m": 0.2},
        {"step": 2, "skill": "pick", "target": puck, "relation": "", "offset_m": 0.0},
        {"step": 3, "skill": "navigate", "target": goal, "relation": "front", "offset_m": 0.2},
        {"step": 4, "skill": "place", "target": goal, "relation": "on", "offset_m": 0.0},
    ], None, ""
