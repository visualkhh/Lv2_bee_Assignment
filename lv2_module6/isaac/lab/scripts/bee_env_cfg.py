"""BEE puck loading env for Isaac Lab (step 4-1: scene only).

Skeleton based on Isaac Lab cartpole_manager_env_cfg.py.
"""

import math
from pathlib import Path

import gymnasium as gym
import torch
import warp as wp
from isaaclab_physx.physics import PhysxCfg

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.visualizers import VisualizerCfg

USD_DIR = Path(__file__).resolve().parents[2] / "sim" / "usda"
URDF_PATH = Path(__file__).resolve().parents[3] / "urdf" / "beebot_mesh" / "beebot.urdf"  # same kinematics as the USD
# end_effector_link relative to gripper_link (merged away in Lab, fixed joint)
EE_OFFSET_POS = (0.109055, 0.0, 0.0)
EE_OFFSET_ROT = (0.0, 0.7071068, 0.0, 0.7071068)  # xyzw
PUCK_USDS = [
    str(USD_DIR / "puck" / "box" / "puck_box_green_0.usda"),
    str(USD_DIR / "puck" / "box" / "puck_box_red_1.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_green_2.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_red_3.usda"),
]
SHELF_HALF = 0.205 / 2  # mission start box 205 x 205 x 70 (shelf.usda is 207 wide, scaled down in the scene)
SHELF_H = 0.07
SHELF_TOP = SHELF_H  # shelf on the floor
SHELF_NEAR_X = 0.08  # near edge, robot front at 0.069
# Puck reset range (robot root frame), measured with pick_sm.py:
#   x < 0.12  the arm is blocked folding down close to the body
#   r > 0.22  servo torque limit with the arm stretched (corner (0.20, 0.08) is r = 0.215)
# The robot stands so the puck lands in this window (nearest shelf side, back off for pucks near the edge).
PUCK_X = 0.16
PUCK_X_RANGE = (0.12, 0.20)
PUCK_Y_RANGE = (-0.08, 0.08)


##
# Robot
##

beebotCFG = ArticulationCfg(
    
    spawn = sim_utils.UsdFileCfg(
        usd_path = str (USD_DIR / "beebot" / "beebot.usda"),
        variants={"Physics": "physx"},
        articulation_props = sim_utils.ArticulationRootPropertiesCfg(fix_root_link=False),
        # PhysX contacts are points, so a two-finger pinch has no torsional friction and the puck spins
        # about the finger axis when the wrist turns. The rubber pads have area; a patch radius models it.
        # ponytail: applied to every robot collider; move it onto the finger colliders if wheels care.
        collision_props = sim_utils.CollisionPropertiesCfg(torsional_patch_radius=0.01, min_torsional_patch_radius=0.005),
    ),
    
    init_state = ArticulationCfg.InitialStateCfg(
        pos = (0, 0, 0),
        # home pose (deg): 0, -110, 80, 19, 0. joint2/joint3 sit on their limits (-110 / 80), so they are
        # pulled 0.001 rad (0.06 deg) inside: Lab rejects defaults that float rounding puts past the limit
        joint_pos = {"joint1": 0.0, "joint2": math.radians(-110.0) + 0.001, "joint3": math.radians(80.0) - 0.001, "joint4": math.radians(19.0), "joint5": 0.0, "gripper_.*_joint": 0.0}
    ),
    
    actuators = {
        "arm": ImplicitActuatorCfg(joint_names_expr =["joint[1-5]"], stiffness = None, damping = None),
        "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*_joint"], stiffness = None, damping = None),
        # 8.3 N per finger on a 30 mm puck: stiffness x stop position (6.2 mm, pick_sm.GRIP_STOP)
        "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_.*_joint"], stiffness = 8.3 / 0.0062, damping = 10.0)
    }
)


##
# Scene definition
##


@configclass
class BeeSceneCfg(InteractiveSceneCfg):
    """Robot, shelf, puck, ground, light."""

    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/DomeLight", spawn=sim_utils.DomeLightCfg(intensity=1000.0, exposure=1.25))

    robot: ArticulationCfg = beebotCFG.replace(prim_path="{ENV_REGEX_NS}/beebot")

    # shelf/puck geo is baked Z-up with origin at bottom center, so init_state.rot stays identity
    shelf = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/shelf",
        spawn = sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "shelf" / "shelf.usda"),
            scale=(0.205 / 0.207, 0.205 / 0.207, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
        init_state = RigidObjectCfg.InitialStateCfg(pos=(SHELF_NEAR_X + SHELF_HALF, 0.0, SHELF_TOP - SHELF_H))
    )
    puck = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/puck",
        spawn = sim_utils.MultiUsdFileCfg(usd_path=PUCK_USDS),
        # 3 x 3 x 6 cm, dropped 2 mm onto the shelf top. x 0.17 = top-down grasp reach limit is about 0.22
        init_state = RigidObjectCfg.InitialStateCfg(pos=(PUCK_X, 0.0, SHELF_TOP + 0.002))
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Joint position targets (zero action = default pose)."""

    arm = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint[1-5]"], scale=1.0)
    gripper = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["gripper_.*_joint"], scale=0.0265)
    

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


def randomize_mass_synced(env, env_ids, asset_cfg: SceneEntityCfg, scale_range: tuple[float, float]):
    """Scale body mass and inertia per env (uniform), like mdp.randomize_rigid_body_mass with operation="scale".

    Lab 6.1 (PhysX) set_masses_index copies the masses and env ids GPU -> pinned CPU asynchronously and hands
    the CPU buffers to PhysX without a sync. At startup PhysX then sometimes reads them before the copy is done:
    env 0 gets mass 0 (inverse mass 0 = infinite mass, so the puck falls through the shelf and the floor) and
    the other envs keep the unrandomized mass. Seen in about half of the runs, always env 0 (2026-10-27).
    Here the arrays are built on the CPU and written synchronously; Lab's GPU buffers are updated to match.
    """
    asset = env.scene[asset_cfg.name]
    n = env.scene.num_envs
    m0 = asset.data.body_mass.torch.clone()  # (n, bodies), USD values at startup
    i0 = asset.data.body_inertia.torch.clone()  # (n, bodies, 9)
    lo, hi = scale_range
    scale = lo + (hi - lo) * torch.rand(n, 1, device=m0.device)
    mass, inertia = (m0 * scale).float(), (i0 * scale[..., None]).float()
    ids = wp.from_torch(torch.arange(n, dtype=torch.int32))
    wp.synchronize()
    asset.root_view.set_masses(wp.from_torch(mass.cpu().contiguous()), indices=ids)
    asset.root_view.set_inertias(wp.from_torch(inertia.cpu().reshape(-1).contiguous()), indices=ids)
    wp.copy(asset.data._body_mass, wp.from_torch(mass.contiguous()))
    wp.copy(asset.data._body_inertia, wp.from_torch(inertia.contiguous()))


@configclass
class EventCfg:
    """Domain randomization. Startup terms are sampled once per env, reset terms every episode.
    Order matters: reset_all runs first, then reset_puck overrides the puck pose."""

    # startup: physical properties
    puck_mass = EventTerm(  # not mdp.randomize_rigid_body_mass: see randomize_mass_synced
        func=randomize_mass_synced,
        mode="startup",
        params={"asset_cfg": SceneEntityCfg("puck"), "scale_range": (0.7, 1.3)},
    )
    puck_material = EventTerm(  # PLA on cardboard; matters for sliding on the shelf
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("puck"),
            "static_friction_range": (0.3, 0.6),
            "dynamic_friction_range": (0.25, 0.55),
            "restitution_range": (0.0, 0.4),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )
    finger_material = EventTerm(  # rubber pad, combine mode "max" so this sets the grip friction
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="finger_.*_link"),
            "static_friction_range": (0.6, 1.0),
            "dynamic_friction_range": (0.5, 0.9),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )

    # reset
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    reset_puck = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("puck"),
            "pose_range": {
                "x": (PUCK_X_RANGE[0] - PUCK_X, PUCK_X_RANGE[1] - PUCK_X),
                "y": PUCK_Y_RANGE,
                "yaw": (-math.pi, math.pi),
            },
            "velocity_range": {},
        },
    )


@configclass
class RewardsCfg:
    """Placeholder until step 4-5."""

    alive = RewTerm(func=mdp.is_alive, weight=0.0)


@configclass
class TerminationsCfg:
    """Scenario timeout only."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


##
# Environment configuration
##


@configclass
class BeeEnvCfg(ManagerBasedRLEnvCfg):
    """BEE puck loading environment."""

    scene: BeeSceneCfg = BeeSceneCfg(num_envs=64, env_spacing=2.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 2
        self.episode_length_s = 20.0
        self.sim.default_visualizer_cfg = VisualizerCfg(eye=(1.2, 1.2, 0.8), lookat=(0.3, 0.0, 0.2))
        self.sim.dt = 1 / 200  # 100 Hz control (decimation 2), the rate chosen in the 2026-10-27 report (4)
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg()  # default backend is Newton; robot uses the physx variant


##
# Step 4-3: IK variant (base motion / state machine drives end effector poses)
##


@configclass
class BeeIKEnvCfg(BeeEnvCfg):
    """Arm takes an end effector pose command (IK), gripper takes open/close."""

    def __post_init__(self) -> None:
        super().__post_init__()

        self.actions.arm = DifferentialInverseKinematicsActionCfg(
            asset_name = "robot",
            joint_names = ["joint[1-5]"],
            body_name = "gripper_link",
            body_offset = DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=EE_OFFSET_POS, rot=EE_OFFSET_ROT),
            controller = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
            )

        self.actions.gripper = mdp.BinaryJointPositionActionCfg(
            asset_name = "robot",
            joint_names = ["gripper_.*_joint"],
            open_command_expr = {"gripper_.*_joint": 0.0265},
            close_command_expr = {"gripper_.*_joint": 0.0}
            )


gym.register(
    id="Bee-Puck-Load",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:BeeEnvCfg"},
)

gym.register(
    id="Bee-Puck-Load-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:BeeIKEnvCfg"},
)
