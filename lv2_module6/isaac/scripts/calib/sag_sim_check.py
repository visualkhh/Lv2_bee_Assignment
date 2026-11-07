"""Dry run of `sag_calib` in Isaac Lab: the same joint-space moves, checked for collisions and speed before the real arm runs them.

Per pose: start pose (set_center, all joints 0) -> pre-pose (joint2-4 +approach_deg) -> pose -> pre-pose (-approach_deg) -> pose,
every move like sag_calib / JTC: one point, zero end velocity (cubic smooth step), duration = max over joints of
max(move_s, |delta| / (0.3 x URDF velocity limit)). The scene has the 200 x 200 x 70 mm shelf 50 mm in front of the chassis.
Reports per move: duration, peak commanded joint speed (analytic, 1.5 x mean) against the URDF limit, peak speed and tracking
error in the sim, and contact forces on the arm bodies (shelf / everything else = self or chassis).

  ~/pa-course/IsaacLab-6.1/.venv/bin/python sag_sim_check.py                       # headless, shelf poses
  ... sag_sim_check.py --poses ros2_ws/src/sine_test/config/sag_poses_basket.yaml  # basket poses
  ... sag_sim_check.py --viz kit                                                    # watch it
Exit code 1 when any contact above --force_n or any speed above the URDF limit is found.
"""

import argparse
import math
import os
import re
from pathlib import Path

import yaml
from isaaclab.app import AppLauncher

HERE = Path(__file__).resolve().parent
CFG = HERE / "ros2_ws" / "src" / "sine_test" / "config"
USD = HERE.parent / "usda" / "beebot" / "beebot.usda"
URDF = HERE.parents[2] / "urdf" / "default" / "urdf.xacro"

parser = argparse.ArgumentParser()
parser.add_argument("--poses", default=str(CFG / "sag_poses.yaml"))
parser.add_argument("--force_n", type=float, default=0.5, help="contact force on an arm body that counts as a collision [N]")
parser.add_argument("--hold_s", type=float, default=1.0, help="stand still at each pose (settle + average window in sag_calib)")
parser.add_argument("--only", default=None, help="comma separated pose names")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

DT = 1 / 200
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
CHASSIS_FRONT, GAP, SHELF = 0.069, 0.05, (0.20, 0.20, 0.07)  # shelf near edge 50 mm in front of the chassis front (root frame x)


def urdf_limits():
    """{joint: velocity limit rad/s} for the arm joints (xacro text, no xacro run)."""
    text = URDF.read_text()
    return {j: float(re.search(rf'<joint name="{j}".*?velocity="([\d.]+)"', text, re.S).group(1)) for j in ARM}


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD), variants={"Physics": "physx"}, activate_contact_sensors=True,
                                   articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=False)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 0), joint_pos={"joint[1-5]": 0.0, "gripper_.*_joint": 0.0}),
        actuators={"arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-5]"], stiffness=None, damping=None),
                   "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*_joint"], stiffness=None, damping=None),
                   "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_.*_joint"], stiffness=None, damping=None)})
    shelf = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/shelf",
        spawn=sim_utils.CuboidCfg(size=SHELF, rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                                  collision_props=sim_utils.CollisionPropertiesCfg(),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.6, 0.3)),
                                  activate_contact_sensors=True),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(CHASSIS_FRONT + GAP + SHELF[0] / 2, 0.0, SHELF[2] / 2)))
    contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/robot/.*", filter_prim_paths_expr=["{ENV_REGEX_NS}/shelf"], history_length=1)


def tt(x):
    return x.torch if hasattr(x, "torch") else x


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def main():
    cfg = yaml.safe_load(Path(args.poses).read_text())
    vlim = urdf_limits()
    approach = math.radians(cfg["approach_deg"])
    names = args.only.split(",") if args.only else list(cfg["poses"])
    poses = {n: np.radians([cfg["poses"][n][j] for j in ARM]) for n in names}

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device))
    sim.set_camera_view(eye=(0.9, -0.9, 0.7), target=(0.1, 0.0, 0.15))
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3.0))
    sim.reset()
    robot, sensor = scene["robot"], scene["contacts"]
    jid = robot.find_joints("joint[1-5]", preserve_order=True)[0]
    bodies = sensor.body_names
    # arm side only: a hit on the pole or chassis from the arm shows on the arm body
    arm_b = [i for i, n in enumerate(bodies) if n not in ("base_footprint", "base_link", "pole_front_cam_link") and "wheel" not in n and "caster" not in n]
    print("[check] sensor bodies:", [bodies[i] for i in arm_b], flush=True)
    q = torch.zeros(1, 5, device=sim.device)
    tgt = lambda a: torch.tensor(a, dtype=torch.float32, device=sim.device)[None]

    def settle(seconds):
        for _ in range(int(seconds / DT)):
            robot.set_joint_position_target_index(target=q, joint_ids=jid)
            scene.write_data_to_sim(); sim.step(); scene.update(DT)

    jp = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp))
    scene.reset(); settle(1.0)

    def forces():
        """(net force, force from the shelf) per sensor body [N]."""
        net = tt(sensor.data.net_forces_w)[0].norm(dim=-1)
        mat = tt(sensor.data.force_matrix_w)
        return net, (mat[0].norm(dim=-1).sum(-1) if mat is not None else torch.zeros_like(net))

    base = torch.zeros(len(bodies), device=sim.device)  # resting contacts at the start pose (arm parts touching the chassis)
    for _ in range(int(0.5 / DT)):
        settle(DT)
        base = torch.maximum(base, forces()[0])
    print("[check] resting contact force [N]:", {bodies[i]: round(base[i].item(), 2) for i in arm_b if base[i] > 0.01}, flush=True)

    bad, rows = [], []
    cur = np.zeros(5)

    def move(goal, label):
        nonlocal cur
        delta = goal - cur
        t = max(max(cfg["move_s"], abs(d) / (0.3 * vlim[j])) for d, j in zip(delta, ARM))
        peak_cmd = 1.5 * np.abs(delta).max() / t
        peak_lim = max(1.5 * abs(d) / t / vlim[j] for d, j in zip(delta, ARM))  # fraction of the URDF limit
        f_shelf = f_other = v_peak = e_peak = 0.0
        hit = set()
        for k in range(int(t / DT)):
            s = smooth((k + 1) * DT / t)
            q[:] = tgt(cur + s * delta)
            robot.set_joint_position_target_index(target=q, joint_ids=jid)
            scene.write_data_to_sim(); sim.step(); scene.update(DT)
            v_peak = max(v_peak, tt(robot.data.joint_vel)[0, jid].abs().max().item())
            e_peak = max(e_peak, (tt(robot.data.joint_pos)[0, jid] - q[0]).abs().max().item())
            net, sh = forces()
            for i in arm_b:
                fs, fo = sh[i].item(), max(net[i].item() - sh[i].item() - base[i].item(), 0.0)
                f_shelf, f_other = max(f_shelf, fs), max(f_other, fo)
                if fs > args.force_n: hit.add(f"{bodies[i]}<->shelf")
                elif fo > args.force_n: hit.add(f"{bodies[i]}<->other")
        cur = goal.copy()
        settle(args.hold_s if label.endswith("pose") else 0.0)
        ok = not hit and peak_lim <= 1.0
        rows.append((label, t, peak_cmd, peak_lim, v_peak, math.degrees(e_peak), f_shelf, f_other, sorted(hit)))
        print(f"[check] {'ok ' if ok else 'BAD'} {label:28s} {t:5.1f} s  cmd peak {peak_cmd:4.2f} rad/s ({peak_lim * 100:3.0f} % of limit)  "
              f"sim peak {v_peak:4.2f} rad/s  track err {math.degrees(e_peak):4.1f} deg  shelf {f_shelf:5.2f} N  other {f_other:5.2f} N  {sorted(hit) or ''}", flush=True)
        if not ok: bad.append(label)

    for n, p in poses.items():
        pre = lambda sign: p + sign * approach * np.array([0, 1, 1, 1, 0])
        for sign, tag in ((+1, "above"), (-1, "below")):
            move(pre(sign), f"{n}/{tag} pre")
            move(p, f"{n}/{tag} pose")
        move(np.zeros(5), f"{n} back to center")
    print(f"[check] {len(rows)} moves, {len(bad)} flagged" + (": " + ", ".join(bad) if bad else ""), flush=True)
    os._exit(1 if bad else 0)  # skips the Kit shutdown (app.close() exits 0 on its own)


if __name__ == "__main__":
    main()
