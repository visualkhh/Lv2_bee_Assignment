import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)

args = parser.parse_args()
simulation_app = AppLauncher(args).app

import gymnasim as gym
import torch
import isaaclab.utils.math as math_utils
import parse_env_cfg
import bee_env_cfg
