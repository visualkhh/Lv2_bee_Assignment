#!/usr/bin/env python3
"""세우기(stand) 경유점 표(방법 A) 내보내기 — 팀원 isaac/rl/bee_isaac/tasks/right_env.py 표와 같은 것을 skills 표 yaml로 (skill_stand가 launch에서 받음).

right_env.RightEnv가 학습 때 푸는 표(모양 × 퍽 축 방향 YAW_BINS × 옆 위치 YCS, 다시 잡기 2개 포함)를 그대로 만든다.
right_env.plan_waypoints · contact_h와 상수를 소스에서 그대로 꺼내 쓰므로 Isaac Lab 없이 numpy + pinocchio만 있으면 된다.

    python3 stand_table_export.py <feature/isaac-lab의 lv2_module6/isaac/lab> [--out stand_table.yaml]
    → skills/params/stand_table.yaml 로 두면 skills.launch.py가 skill_stand에 넘김 · 쓰려면 skills.yaml stand_waypoints: table

값의 배열 순서 (skills/stand_plan.hpp Table): q[ycs][yaw_bins][W][5], P[ycs][yaw_bins][W][5][3] (평탄화)
"""
import argparse
import ast
import math
import sys
from pathlib import Path

import numpy as np


def load_right_env(lab):
    lab = Path(lab)  # 옛 브랜치 isaac/lab 또는 지금 isaac/rl/bee_isaac/tasks
    if (lab / "bee_common.py").exists():
        sys.path.insert(0, str(lab))
        import bee_common as bc  # numpy + pinocchio
    else:
        sys.path.insert(0, str(lab.parents[1]))
        from bee_isaac import common as bc  # numpy + pinocchio
    import pinocchio as pin

    ns = {"np": np, "math": math, "bc": bc, "pin": pin}
    keep = {"YAW_BINS", "PHIS", "PUCK_L", "PUCK_W", "YCS"}
    tree = ast.parse((Path(lab) / "right_env.py").read_text())
    for node in tree.body:
        names = {n.id for t in getattr(node, "targets", []) for n in ast.walk(t) if isinstance(n, ast.Name)}
        if (isinstance(node, ast.Assign) and names & keep) or (
                isinstance(node, ast.FunctionDef) and node.name in ("contact_h", "plan_waypoints")):
            exec(compile(ast.Module([node], []), "right_env.py", "exec"), ns)
    pe_tree = ast.parse((Path(lab) / "place_env.py").read_text())  # BOX_CX = BOX_NEAR_X + bc.BOX_HALF
    for node in pe_tree.body:
        names = {n.id for t in getattr(node, "targets", []) for n in ast.walk(t) if isinstance(n, ast.Name)}
        if isinstance(node, ast.Assign) and names & {"BOX_NEAR_X", "BOX_CX"}:
            exec(compile(ast.Module([node], []), "place_env.py", "exec"), ns)
    return bc, ns


class Cfg:  # right_env.RightEnvCfg (tilt) — 값이 바뀌면 여기와 skill_stand stand_* 파라미터를 같이
    mode = "tilt"
    above_m, lift_away_m = 0.04, 0.08
    pinch_from_end_m, press_m = 0.0, 0.0
    contact_h_m, contact_h_cyl_m = 0.030, 0.015
    tilt_over_m, arc_end_deg, set_dz_m = 0.010, 120.0, -0.010
    regrasp, regrasp_lift_m = 1, 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lab", help="feature/isaac-lab의 lv2_module6/isaac/lab")
    ap.add_argument("--out", default="stand_table.yaml")
    ap.add_argument("--source", default="", help="출처 기록 (예: feature/isaac-lab 커밋)")
    a = ap.parse_args()
    bc, ns = load_right_env(a.lab)
    ik = bc.ArmIK()
    cfg = Cfg()
    tabs = {}
    for k, shape in ((0, "cyl"), (1, "box")):
        qs, Ps = [], []
        for yc in ns["YCS"]:
            for yw in ns["YAW_BINS"]:
                q, P = ns["plan_waypoints"](ik, np.array([ns["BOX_CX"], yc, bc.BOX_H]) - ik.base_in_root(), yw, cfg, is_box=k == 1)
                qs.append(np.array(q))
                Ps.append(np.array(P))
        tabs[shape] = (np.array(qs), np.array(Ps))
    W = tabs["box"][0].shape[1]
    fl = lambda v: "[" + ", ".join(f"{x:.6f}" for x in np.asarray(v).ravel()) + "]"
    with open(a.out, "w") as f:
        f.write("# 세우기 경유점 표 (방법 A) — skills/scripts/stand_table_export.py 로 만듦, 손으로 고치지 않음\n")
        f.write("# 출처: feature/isaac-lab isaac/lab/right_env.py plan_waypoints (tilt) · 순서 q[ycs][yaw][W][5] · P[ycs][yaw][W][5][3]\n")
        if a.source:
            f.write(f"# 출처 커밋: {a.source}\n")
        f.write(f"stand_table_yaw_deg: {fl(np.degrees(ns['YAW_BINS']))}\n")
        f.write(f"stand_table_ycs: {fl(ns['YCS'])}\n")
        f.write(f"stand_table_box_cx: {ns['BOX_CX']:.6f}\nstand_table_box_h: {bc.BOX_H:.6f}\nstand_table_w: {W}\n")
        for shape in ("cyl", "box"):
            f.write(f"stand_table_{shape}_q: {fl(tabs[shape][0])}\nstand_table_{shape}_p: {fl(tabs[shape][1])}\n")
    print(f"{a.out}: {len(ns['YCS'])} ycs x {len(ns['YAW_BINS'])} yaw x W {W}")


if __name__ == "__main__":
    main()
