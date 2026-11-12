"""Paths and layout values shared by Isaac Lab, standalone Isaac and ROS scripts. Standard library only: no Isaac, numpy or ROS."""

from pathlib import Path

ISAAC_DIR = Path(__file__).resolve().parents[2]  # lv2_module6/isaac
ROOT = ISAAC_DIR.parent                          # lv2_module6
USD_DIR = ISAAC_DIR / "usda"
SOURCE_DIR = ISAAC_DIR / "source"
POLICY_DIR = ISAAC_DIR / "policies"
LOG_DIR = ISAAC_DIR / "logs"
DATA_DIR = Path(__file__).resolve().parent / "data"
CUMOTION_DIR = ISAAC_DIR / "cumotion"
URDF_PATH = ROOT / "urdf" / "beebot_mesh" / "beebot.urdf"  # same kinematics as the USD
LEARNING_PLAN_DIR = ROOT / "learning" / "plan"

PUCK_USDS = [
    str(USD_DIR / "puck" / "box" / "puck_box_green_0.usda"),
    str(USD_DIR / "puck" / "box" / "puck_box_red_1.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_green_2.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_red_3.usda"),
]

# scenes/manipulator_scene.py: base_footprint at the origin, +x towards the box
BOX_HALF = 0.1025     # plan.yaml box_half
ROBOT_TO_BOX = 0.30   # base_footprint to the box edge: pole_front_cam sees both pucks from here (not from 0.10)
BOX_X = ROBOT_TO_BOX + BOX_HALF
PUCK_DX = 0.04        # plan.yaml puck_offsets_x
PUCK_Z = 0.0705       # puck origin height in scene_sim
PUCK_L, PUCK_W = 0.06, 0.03  # standing height, width (puck origin = centre of its bottom end)
PUCK_XY = [(BOX_X - PUCK_DX, 0.0), (BOX_X + PUCK_DX, 0.0)]
