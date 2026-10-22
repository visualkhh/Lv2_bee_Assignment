"""Run a BEE env with zero actions (robot holds its default pose).

Copy of Isaac Lab scripts/environments/zero_agent.py plus the BEE env registration.

    isaaclab.sh -p zero_agent.py --task Bee-Puck-Load --num_envs 4
"""

import bee_env_cfg
from isaaclab_rl.entrypoints import run_random_agent_cli

if __name__ == "__main__":
    raise SystemExit(run_random_agent_cli())
