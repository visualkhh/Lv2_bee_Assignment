"""fmt_pose — 지도 화면이 goal·위치를 글자로 보여 줄 때 쓴다."""


def fmt_pose(p: dict | None) -> str:
    return '지금 위치 (실행할 때)' if p is None else f'({p["x"]:+.2f}, {p["y"]:+.2f}) {p["yaw_deg"]:+.0f}°'
