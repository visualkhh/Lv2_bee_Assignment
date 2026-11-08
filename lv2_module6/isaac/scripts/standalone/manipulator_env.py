"""BEE 집기 시연 환경 (standalone) — scene_sim.usda를 열지 않고 stage를 직접 구성: 바닥 · 시작 상자 1개 · 그 위 퍽 2개 · 로봇.

  isaac/scripts/demo/pick_load_demo.sh   (이 스크립트 + ROS까지 — isaac_sim.sh 경유)
  장면은 bee_isaac.scenes.manipulator_scene · 배치는 bee_isaac.constants
  에셋(beebot · shelf · puck)은 isaac/usda 파일을 참조로 씀 — 로봇 안 ActionGraph(/clock · 카메라)와 opencr_bridge.py(가상 OpenCR)는 그대로
"""
import argparse
import json
import math
import os
import time

from bee_isaac.scenes.manipulator_layout import layout, random_layout  # 표준 라이브러리만 — Isaac 전에 import 가능

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-play", action="store_true", help="Play를 누르지 않음")
parser.add_argument("--puck", action="append", metavar="색:모양:상태:태그",
                    help="퍽 하나, 2번 반복: red|green : box|cyl : stand|lie : 0~11. 뒤쪽·빈 항목은 --seed로 정함 (예: red:cyl:stand:3, green::lie)")
parser.add_argument("--seed", type=int, help="--puck에서 비운 항목과 퍽 방향을 정함. 없으면 방향은 scene_sim 값")
parser.add_argument("--random-layout", action="store_true",
                    help="상자를 차체 앞 2~12 cm에, 퍽 2개(서 있음)를 상자 위 아무 데나 — 리셋마다 새로 뽑음 (seed는 /tmp/beebot_trial_seed, 없으면 +1). 결과는 /tmp/beebot_trial.json")
parser.add_argument("--fix-base", action="store_true", help="로봇 본체를 월드에 고정 (팔이 닿아도 안 밀림 — 순수 pick 시험용. 차체 기울기·밀림은 재현 안 됨)")
parser.add_argument("--gripper-view", action="store_true", help="뷰포트를 gripper_cam_rgb 시점으로")
parser.add_argument("--arm-q", help="팔 시작 자세 joint1~5 [deg], 예: 0,-96.4,18.6,110.4,0 · box = 시행마다 상자 윗면 가운데를 보는 자세 "
                                     "(tools/inspect_pose.py look_at) · cover = 간격별 상자 위를 가장 많이 덮는 자세 (tools/inspect_cover.py 표). 쓴 값은 /tmp/beebot_arm_q (isaac_perception_only.launch.py가 읽음)")
parser.add_argument("--trial-seed", type=int, default=0, help="--random-layout 첫 시행의 seed")
args = parser.parse_args()
trial = None
try:
    if args.random_layout:
        trial = random_layout(args.trial_seed)
        pucks = trial.pucks
    else:
        pucks = layout(args.puck, args.seed)
except ValueError as e:
    parser.error(str(e))  # Isaac을 띄우기 전에 끝냄

from isaacsim import SimulationApp  # noqa: E402 — 다른 omni import보다 먼저

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.experimental.physics", "isaacsim.core.nodes"):
    enable_extension(ext)
app.update()

from bee_isaac.constants import BOX_X  # noqa: E402
from bee_isaac.scenes.manipulator_scene import VIEW_CAMERA, build, move, put_pucks  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESET_FILE = "/tmp/beebot_isaac_reset"  # isaac_sim.sh와 같은 경로
SLOT_FILE = "/tmp/beebot_slot_pucks.json"  # pick_load_play.py가 읽는 적재 판정 입력 (같은 경로)
SLOT_EVERY = 15  # app.update 몇 번마다 쓸지
TRIAL_FILE = "/tmp/beebot_trial.json"  # --random-layout: 지금 시행의 배치 (pick_test.py가 읽음)
SEED_FILE = "/tmp/beebot_trial_seed"   # pick_test.py가 리셋 전에 다음 seed를 적음
SOFT_FILE = "/tmp/beebot_soft_reset"   # pick_test.py: 이 파일이 생기면 Stop 없이 장면만 새 배치로 (ROS·시계 그대로)
FINGER_FACE_X = 0.109055 + 0.0206      # gripper_link 안 손가락 끝면 (end_effector_link + FINGER_TIP, stand_plan.hpp)
BASE = "/World/beebot/Geometry/base_footprint/base_link"  # 관절 루트 = 적재함 칸 basket_slot_*의 부모

omni.usd.get_context().new_stage()
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
build(stage, pucks, (*trial.box, trial.box_yaw) if trial else (BOX_X, 0.0, 0.0), trial)
for _ in range(60):  # 페이로드 · 그래프가 올라올 때까지
    app.update()
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport  # noqa: E402

    get_active_viewport().camera_path = (next(p.GetPath() for p in stage.Traverse() if p.GetName() == "gripper_cam_rgb")
                                         if args.gripper_view else VIEW_CAMERA)

if args.fix_base:  # Play 전에: 월드 → base_link 고정 조인트 (body0 비움 = 월드). 지금 자세에 그대로 고정
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    tf = UsdGeom.Xformable(stage.GetPrimAtPath(BASE)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    joint = UsdPhysics.FixedJoint.Define(stage, "/World/base_fix")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(BASE)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*tf.ExtractTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(tf.ExtractRotationQuat()))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    print(f"[manipulator_env] 본체 고정 — base_link {tuple(round(v, 3) for v in tf.ExtractTranslation())}", flush=True)

ARM_Q_FILE = "/tmp/beebot_arm_q"  # 지금 팔 시작 자세 [deg] — isaac_perception_only.launch.py의 joint_states


def set_arm():
    """Play 전에 (Stop 상태): joint1~5 시작 자세 = drive 목표 (opencr_bridge는 명령이 없으면 그대로 유지)."""
    if not args.arm_q:
        return
    if args.arm_q == "box":
        import math
        import sys
        sys.path.insert(0, os.path.join(HERE, "..", "tools"))
        from inspect_pose import look_at
        q, info = look_at((trial.box[0], trial.box[1], 0.13))
        if q is None:
            print(f"[manipulator_env] 상자 보는 자세 없음: {info} — 팔 그대로", flush=True)
            return
        deg = [math.degrees(v) for v in q]
    elif args.arm_q == "cover":  # 간격별로 상자 위를 가장 많이 덮는 자세 표 (tools/inspect_cover.py --table)
        with open(os.path.join(HERE, "..", "tools", "inspect_cover_table.json")) as f:
            table = json.load(f)
        deg = table[min(table, key=lambda g: abs(float(g) - trial.gap))]["q_deg"]
    else:
        deg = [float(v) for v in args.arm_q.split(",")]
    joints = {p.GetName(): p for p in stage.Traverse() if p.GetName() in ("joint1", "joint2", "joint3", "joint4", "joint5")}
    for k, d in enumerate(deg):
        j = joints[f"joint{k + 1}"]
        j.GetAttribute("state:angular:physics:position").Set(d)
        j.GetAttribute("drive:angular:physics:targetPosition").Set(d)
    with open(ARM_Q_FILE, "w") as f:
        f.write(",".join(f"{d:.4f}" for d in deg))
    print(f"[manipulator_env] 팔 시작 자세 {[round(d, 1) for d in deg]} deg", flush=True)


set_arm()

bridge = os.path.join(HERE, "opencr_bridge.py")  # pty를 열고, 첫 물리 스텝에서 로봇에 붙어 /tmp/opencr를 만듦
exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})

import omni.physics.tensors  # noqa: E402
from pxr import Gf  # noqa: E402

SLOTS = [stage.GetPrimAtPath(f"{BASE}/basket_slot_{i}").GetAttribute("xformOp:translate").Get() for i in range(12)]
views = {}
BASE0 = None  # 처음 본체 자세 [x y z qx qy qz qw] — 소프트 리셋 때 되돌림 (손이 퍽·상자를 쳐 본체가 밀리면 그 뒤 시행이 전부 어긋남, 11-07 실측)


def write_trial():
    rows = [{"name": p.name, "shape": p.shape, "tag": p.tag, "x": p.pos[0], "y": p.pos[1], "yaw": p.rot[2], "dist": trial.dist[k],
             "reachable": trial.reachable[k], "mass_scale": trial.mass[k], "friction_scale": trial.friction[k]} for k, p in enumerate(pucks)]
    with open(TRIAL_FILE + ".tmp", "w") as f:
        json.dump({"seed": trial.seed, "t": time.time(), "gap": trial.gap, "box": trial.box, "box_yaw": trial.box_yaw, "pucks": rows}, f)
    os.replace(TRIAL_FILE + ".tmp", TRIAL_FILE)


def reset_base():
    """본체를 처음 자세로 · 속도 0 (Stop 없이). views가 아직 유효할 때(장면을 바꾸기 전) 부른다."""
    if args.fix_base or BASE0 is None or not views or "robot" not in views:  # 고정 모드에선 필요 없음
        return
    import numpy as np
    idx = np.array([0], dtype=np.int32)
    try:  # Stop 상태나 루트 링크가 안 받는 경우 "Failed to set root link transforms" — 그래도 시행은 계속
        views["robot"].set_root_transforms(np.array([BASE0], dtype=np.float32), idx)
        views["robot"].set_root_velocities(np.zeros((1, 6), dtype=np.float32), idx)
    except Exception as e:  # noqa: BLE001
        print(f"[manipulator_env] 본체 복귀 못 함: {e}", flush=True)


def next_trial():
    """Reset in --random-layout mode (timeline stopped): new seed → box and pucks of that trial."""
    global trial, pucks
    reset_base()
    try:
        with open(SEED_FILE) as f:
            seed = int(f.read().strip())
        os.remove(SEED_FILE)
    except (OSError, ValueError):
        seed = trial.seed + 1
    trial = random_layout(seed)
    pucks = trial.pucks
    put_pucks(stage, pucks, trial)
    move(stage, "/World/start_box", trial.box[0], y=trial.box[1], rot=(0.0, 0.0, trial.box_yaw))
    for _ in range(40):  # payload가 올라올 때까지
        app.update()
    views.clear()
    set_arm()
    write_trial()
    print(f"[manipulator_env] 시행 seed {seed} — 간격 {trial.gap * 100:.1f} cm · 퍽 {[(p.name, round(d, 3), ok) for p, d, ok in zip(pucks, trial.dist, trial.reachable)]}", flush=True)


def slot_pucks():
    """퍽마다 가장 가까운 적재함 칸과 거리 — base_link 기준, 퍽 원점(바닥 가운데) vs 칸 원점. 물리 정답 위치 (tensor view)"""
    if not views or not views["sim"].is_valid:  # Stop이면 view가 무효 — Play 뒤 다시 만듦 (opencr_bridge와 같음)
        sim = omni.physics.tensors.create_simulation_view("numpy", omni.usd.get_context().get_stage_id())
        views.update(sim=sim, robot=sim.create_articulation_view(BASE),
                     pucks={p.name: sim.create_rigid_body_view(f"/World/{p.name}") for p in pucks})
    b = [float(v) for v in views["robot"].get_root_transforms()[0]]  # x y z qx qy qz qw
    global BASE0
    if BASE0 is None:
        BASE0 = b
    inv = Gf.Quatd(b[6], b[3], b[4], b[5]).GetInverse()
    hand = hand_frame()
    out = {"_base": {"x": b[0], "y": b[1], "z": b[2],  # 로봇 본체 자세 (world) — 소프트 리셋은 본체를 안 되돌리므로 밀림을 봄
                     "yaw": math.degrees(math.atan2(2 * (b[6] * b[5] + b[3] * b[4]), 1 - 2 * (b[4] ** 2 + b[5] ** 2)))}}
    for name, view in views["pucks"].items():
        t = view.get_transforms()[0]
        q = inv.Transform(Gf.Vec3d(float(t[0]) - b[0], float(t[1]) - b[1], float(t[2]) - b[2]))
        dxy, dz, i = min((math.hypot(q[0] - s[0], q[1] - s[1]), q[2] - s[2], i) for i, s in enumerate(SLOTS))
        up_w = Gf.Quatd(float(t[6]), float(t[3]), float(t[4]), float(t[5])).Transform(Gf.Vec3d(0, 0, 1))
        out[name] = {"slot": i, "dxy": dxy, "dz": dz, "z": float(t[2]), "xy": [float(t[0]), float(t[1])],
                     "up_tilt": math.degrees(math.acos(max(-1.0, min(1.0, up_w[2])))),
                     "yaw": math.degrees(math.atan2(2 * (float(t[6]) * float(t[5]) + float(t[3]) * float(t[4])),
                                                    1 - 2 * (float(t[4]) ** 2 + float(t[5]) ** 2)))}
        if hand:  # 퍽 바닥 중심을 손(gripper_link) 좌표로 — x 손끝 방향 · y 닫힘 방향 · z 손가락 길이 방향 (Isaac 위치)
            ih = Gf.Quatd(hand[6], hand[3], hand[4], hand[5]).GetInverse()
            g = ih.Transform(Gf.Vec3d(float(t[0]) - hand[0], float(t[1]) - hand[1], float(t[2]) - hand[2]))
            up = ih.Transform(Gf.Quatd(float(t[6]), float(t[3]), float(t[4]), float(t[5])).Transform(Gf.Vec3d(0, 0, 1)))
            out[name]["hand"] = {"face_above_bottom": g[0] - FINGER_FACE_X, "close_off": g[1], "finger_off": g[2],
                                 "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, -up[0]))))}
    return out


def hand_frame():
    """gripper_link pose [x y z qx qy qz qw] (world) — None if the link cannot be found"""
    try:
        art = views["robot"]
        if "hand_idx" not in views:
            views["hand_idx"] = list(art.shared_metatype.link_names).index("gripper_link")
        return [float(v) for v in art.get_link_transforms()[0][views["hand_idx"]]]
    except Exception as e:  # noqa: BLE001 — 한 번만 알림
        if not views.get("hand_warned"):
            views["hand_warned"] = True
            print(f"[manipulator_env] gripper_link 위치 못 읽음 — 손 기준 값 없음: {e}", flush=True)
        return None


timeline = omni.timeline.get_timeline_interface()
if trial:
    write_trial()
if not args.no_play:
    timeline.play()
print(f"[manipulator_env] 준비 — 상자 가운데 x {BOX_X:.4f} · 퍽 {[(p.name, p.posture, p.pos[:2], p.rot[2]) for p in pucks]}", flush=True)
frame = 0
while app.is_running():
    app.update()
    frame += 1
    if frame % SLOT_EVERY == 0 and timeline.is_playing():
        try:
            rows = slot_pucks()
        except Exception as e:  # noqa: BLE001 — Play 직후 첫 물리 스텝 전이면 view를 못 만듦, 다음에 다시
            views.clear()
            if frame % (SLOT_EVERY * 40) == 0:
                print(f"[manipulator_env] 적재 판정 위치 못 읽음: {e}", flush=True)
        else:
            with open(SLOT_FILE + ".tmp", "w") as f:
                json.dump({"t": time.time(), "pucks": rows}, f)
            os.replace(SLOT_FILE + ".tmp", SLOT_FILE)
    if trial and os.path.exists(SOFT_FILE):  # 시행 사이를 빠르게: 퍽·상자만 새 배치, 시뮬은 계속 돎 → ROS를 다시 안 띄움
        try:
            next_trial()
        except Exception as e:  # noqa: BLE001 — 실패해도 Isaac을 죽이지 않음 (pick_test가 시간 초과로 알림)
            print(f"[manipulator_env] soft reset 실패: {e!r}", flush=True)
        os.remove(SOFT_FILE)
    if os.path.exists(RESET_FILE):  # reset.sh: Stop(처음 자세로) → Play — ROS 재시작은 isaac_sim.sh
        timeline.stop()
        for _ in range(30):
            app.update()
        if trial:
            next_trial()
        timeline.play()
        os.remove(RESET_FILE)
app.close()
