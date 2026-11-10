"""Paths and layout values shared by Isaac Lab, standalone Isaac and ROS scripts. Standard library only: no Isaac, numpy or ROS."""

from pathlib import Path

ISAAC_DIR = Path(__file__).resolve().parents[2]  # lv2_module6/isaac
ROOT = ISAAC_DIR.parent                          # lv2_module6
USD_DIR = ISAAC_DIR / "usda"
POLICY_DIR = ISAAC_DIR / "rl" / "policies"
LOG_DIR = ISAAC_DIR / "rl" / "logs"
DATA_DIR = Path(__file__).resolve().parent / "data"
CUMOTION_DIR = ISAAC_DIR / "cumotion"
URDF_PATH = ROOT / "urdf" / "beebot_mesh" / "beebot.urdf"  # same kinematics as the USD

PUCK_USDS = [
    str(USD_DIR / "puck" / "box" / "puck_box_green_0.usda"),
    str(USD_DIR / "puck" / "box" / "puck_box_red_1.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_green_2.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_red_3.usda"),
]
