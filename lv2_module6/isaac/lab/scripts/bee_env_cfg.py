"""BEE puck loading env for Isaac Lab (step 4-1: scene only).

Skeleton based on Isaac Lab cartpole_manager_env_cfg.py.
"""

from pathlib import Path

import gymnasium as gym
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
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.visualizers import VisualizerCfg

USD_DIR = Path(__file__).resolve().parents[2] / "sim" / "usda"


##
# Robot
##

beebotCFG = ArticulationCfg(
    
    spawn = sim_utils.UsdFileCfg(
        usd_path = str (USD_DIR / "beebot" / "beebot.usda"),
        variants={"Physics": "physx"},
        articulation_props = sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True)
    ),
    
    init_state = ArticulationCfg.InitialStateCfg(
        pos = (0, 0, 0),
        joint_pos = {"joint1": 0.0, "joint2": -1.8675, "joint3": 1.3439, "joint4": 0.5236, "joint5": 0.0, "gripper_.*_joint": 0.0}
    ),
    
    actuators = {
        "arm": ImplicitActuatorCfg(joint_names_expr =["joint[1-5]"], stiffness = None, damping = None),
        "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_.*_joint"], stiffness = None, damping = None),
        "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_.*_joint"], stiffness = 1000.0, damping = 10.0)
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

    shelf = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/shelf",
        spawn = sim_utils.UsdFileCfg(usd_path=str(USD_DIR / "shelf" / "shelf.usda")),
        init_state = RigidObjectCfg.InitialStateCfg(pos=(0.23, 0.0, 0.12))  # top 0.10-0.14 (4-2 reach), near edge x 0.08 (robot front 0.069)
    )
    puck = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/puck",
        spawn = sim_utils.MultiUsdFileCfg(
            usd_path=[str(USD_DIR / "puck" / "puck_red_cyl.usda"), str(USD_DIR / "puck" / "puck_blue_box.usda")],
        ),
        init_state = RigidObjectCfg.InitialStateCfg(pos=(0.14, 0.0, 0.15), rot=(0.0, 0.0, 0.0, 1.0))
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


@configclass
class EventCfg:
    """Reset everything to its initial state."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


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
        self.sim.dt = 1 / 120
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
            body_offset = DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=(0.0, 0.0, 0.109055)),
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
