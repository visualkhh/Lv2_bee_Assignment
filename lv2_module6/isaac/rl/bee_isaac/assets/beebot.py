"""BEEBOT robot config for Isaac Lab tasks and demos. The prim comes from usda/beebot/beebot.usda."""

import math

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from bee_isaac import common as bc


def robot_cfg(grip_force=bc.GRIP_FORCE):
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(bc.USD_DIR / "beebot" / "beebot.usda"),
            variants={"Physics": "physx"},
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=False),
            # two-finger pinch has no torsional friction with point contacts; the rubber pads have area
            collision_props=sim_utils.CollisionPropertiesCfg(torsional_patch_radius=0.01, min_torsional_patch_radius=0.005),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 0),
            joint_pos={"joint1": 0.0, "joint2": math.radians(-110.0) + 0.001, "joint3": math.radians(80.0) - 0.001,
                       "joint4": math.radians(19.0), "joint5": 0.0, "gripper_.*_joint": 0.0},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-5]"], stiffness=None, damping=None),
            "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*_joint"], stiffness=None, damping=None),
            # force per finger = stiffness x (stop position - 0) on a 30 mm puck
            "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_.*_joint"], stiffness=grip_force / bc.GRIP_STOP,
                                           damping=10.0, effort_limit_sim=max(20.0, 2 * grip_force)),
        },
    )
