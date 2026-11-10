"""Train / evaluate a residual policy (Bee-Place-Residual, or Bee-Pick-Residual with --task pick) with rsl_rl PPO.

    python train.py --mode zero  --num_envs 256 --episodes 1024         base motion only (residual 0)
    python train.py --mode train --num_envs 512 --iters 600              train, export policies/place_residual.pt
    python train.py --mode eval  --num_envs 256 --episodes 1024 --policy policies/place_residual.pt
(run with the Isaac Lab venv python: ~/pa-course/IsaacLab-6.1/.venv/bin/python, headless by default)
"""

import argparse
import shutil
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["zero", "train", "eval"], default="zero")
parser.add_argument("--task", help="pick_standing = 서 있는 퍽 집기 (pick_env.py) · stand_lying = 누운 퍽 세우기 (right_env.py); 예전 이름 pick · stand · right도 됨",
                    choices=["place", "pick", "right", "pip", "load", "unload", "reach", "drive", "stand", "pick_standing", "stand_lying"], default="place")
parser.add_argument("--num_envs", type=int, default=4096)  # RTX 5060 Laptop: ~91 % GPU, 4.3 GB (512: 75 %)
parser.add_argument("--iters", type=int, default=600)
parser.add_argument("--episodes", type=int, default=1024, help="zero / eval: finished episodes to collect")
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--policy", default=None, help="eval: TorchScript policy (exported)")
parser.add_argument("--resume", default=None, help="train: rsl_rl checkpoint to continue from")
parser.add_argument("--vel_scale", type=float, default=None)
parser.add_argument("--settle", type=float, default=None)
parser.add_argument("--wait", type=float, default=None, help="stillness check after the hand is up [s]")
parser.add_argument("--pos_noise", type=float, default=None, help="pick: perception position error +- [mm]")
parser.add_argument("--play_speed", type=int, default=1, help="window: draw every N-th policy step (N x faster playback)")
parser.add_argument("--puck", choices=["box", "cyl"], default=None, help="only this puck kind (puck kinds are fixed per env at spawn)")
parser.add_argument("--pinch", type=float, default=None, help="right: pinch force per finger while pivoting [N]")
parser.add_argument("--cfg", default="", help="env cfg overrides, e.g. press_m=0.005,arc_s=3,sag_scale_range=1.5:1.5")
parser.add_argument("--obs_dim", type=int, default=None, help="eval: feed only the first N policy observations (policy trained on a smaller env)")
parser.add_argument("--speed_dr", action="store_true", help="sample vel_scale 0.3-0.8 and settle 0.1-0.3 s per episode")
parser.add_argument("--sag_dr", action="store_true", help="sample the sag integral gain x 0.5-1.5 and the sag x 0.7-1.5 per episode")
parser.add_argument("--out", default=None, help="train: exported file name in policies/ (default <task>_residual.pt)")
parser.add_argument("--run", default=None, help="log folder name under logs/ (default <task>)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.task = {"pick_standing": "pick", "stand_lying": "stand"}.get(args.task, args.task)  # clearer names for the teammate envs
args.out = args.out or f"{args.task}_residual.pt"
args.run = args.run or args.task
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from importlib.metadata import version  # noqa: E402
from isaaclab_rl.rsl_rl import (RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg, RslRlVecEnvWrapper,  # noqa: E402
                                handle_deprecated_rsl_rl_cfg)
from isaaclab.utils import configclass  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from bee_isaac.constants import LOG_DIR, POLICY_DIR  # noqa: E402
from bee_isaac.tasks import pick_env  # noqa: E402
from bee_isaac.tasks import pip_env  # noqa: E402
from bee_isaac.tasks import right_env  # noqa: E402
from bee_isaac.tasks import slot_env  # noqa: E402
from bee_isaac.tasks import reach_env  # noqa: E402
from bee_isaac.tasks import drive_env  # noqa: E402

# 'stand' = 누운 퍽 세우기 — 팀원 환경 이름 'right'(right a puck = 세우다)가 헷갈려서 같은 환경을 이 이름으로도 부름
gym.register(id="Bee-Stand-Residual", entry_point="right_env:RightEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": "right_env:RightEnvCfg"})
from bee_isaac.tasks import place_env  # noqa: E402

HERE = Path(__file__).resolve().parent


@configclass
class PlacePPOCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 600
    save_interval = 50
    experiment_name = "bee_place_residual"
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    clip_actions = 1.0
    actor = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
                             distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.3))
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.002,
                                     num_learning_epochs=5, num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive",
                                     gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0)


def make_env():
    cfg = {"pick": pick_env.PickEnvCfg, "right": right_env.RightEnvCfg, "pip": pip_env.PipEnvCfg,
           "load": slot_env.LoadEnvCfg, "unload": slot_env.UnloadEnvCfg,
           "reach": reach_env.ReachEnvCfg, "drive": drive_env.DriveEnvCfg,
           "stand": right_env.RightEnvCfg}.get(args.task, place_env.PlaceEnvCfg)()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device  # --device cpu: CPU PhysX (another run holds the 8 GB GPU)
    cfg.seed = args.seed
    if args.vel_scale is not None:
        cfg.vel_scale = args.vel_scale
    if args.settle is not None:
        cfg.settle_s = args.settle
    if args.speed_dr:
        cfg.vel_scale_range, cfg.settle_range = (0.3, 0.8), (0.1, 0.3)
    if args.sag_dr:  # ponytail: sag range is a guess until the real sag (B0-0) is measured
        cfg.corr_gain_range, cfg.sag_scale_range = (0.5, 1.5), (0.7, 1.5)
    if args.pos_noise is not None:
        cfg.pos_noise_m = args.pos_noise / 1000
    if args.pinch is not None:
        cfg.pinch_force = args.pinch
    for kv in filter(None, args.cfg.split(",")):
        k, v = kv.split("=")
        setattr(cfg, k, tuple(float(x) for x in v.split(":")) if ":" in v else type(getattr(cfg, k))(v))  # a:b = range
    if args.wait is not None:
        cfg.wait_s = args.wait
    if args.puck:
        cfg.scene.puck.spawn.usd_path = [u for u in cfg.scene.puck.spawn.usd_path if f"/{args.puck}/" in u]
    if args.visualizer:  # window: look at env 0 (box near edge 0.08 m ahead of the robot)
        from isaaclab.visualizers import VisualizerCfg
        cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=(0.6, 0.45, 0.4), lookat=(0.18, 0.0, 0.08))
        cfg.sim.render_interval = cfg.decimation * args.play_speed
    return gym.make(f"Bee-{args.task.capitalize()}-Residual", cfg=cfg)


def report(env, label):
    s = env.unwrapped.stats
    ok, err = np.array(s["success"], dtype=bool), np.array(s["err"]) * 1000
    n_fail = int(np.sum(s["dropped"])) + int(np.sum(s["tipped"]))
    total = len(ok) + n_fail
    print(f"[{label}] episodes {total}: success {ok.sum()}/{total} = {100 * ok.sum() / max(total, 1):.1f} %, "
          f"dropped {int(np.sum(s['dropped']))}, tipped {int(np.sum(s['tipped']))}")
    if "stood" in s:  # right: puck upright at the end of the open phase (regrasp: before the regrasp)
        st = np.array(s["stood"], dtype=bool)
        print(f"[{label}] stood up {st.sum()}/{len(st)} = {100 * st.mean():.1f} % of finished episodes")
    extra = [k for k in ("box_moved", "nb_fell", "fell_over") if k in s]
    if extra:
        print(f"[{label}] early failures: " + ", ".join(f"{k} {int(np.sum(s[k]))}" for k in extra))
    if "by_kind" in s:
        bk = np.array(s["by_kind"], dtype=float)
        for kv, name in ((1.0, "box"), (0.0, "cyl")):
            r = bk[bk[:, 0] == kv]
            if len(r):
                print(f"[{label}] {name}: success {int(r[:, 1].sum())}/{len(r)} = {100 * r[:, 1].mean():.1f} %, "
                      f"squeeze per finger mean {np.median(r[:, 2]):.2f} N (median), max p90 {np.percentile(r[:, 3], 90):.2f} N")
    if "up_max" in s:
        for k in ("up_max", "up_open"):
            v = np.degrees(np.arccos(np.clip(np.array(s[k]), -1, 1)))
            print(f"[{label}] {k} as tilt deg: p10 {np.percentile(v, 10):.0f}, median {np.median(v):.0f}, p90 {np.percentile(v, 90):.0f}")
    if len(err):
        print(f"[{label}] centre error mm: median {np.median(err):.1f}, p90 {np.percentile(err, 90):.1f}, max {err.max():.1f}")


def rollout(env, act_fn, label):
    obs, _ = env.reset()
    u = env.unwrapped
    t0 = time.time()
    k = 0
    while len(u.stats["success"]) + int(np.sum(u.stats["dropped"])) + int(np.sum(u.stats["tipped"])) < args.episodes:
        with torch.inference_mode():
            obs, *_ = env.step(act_fn(obs))
        k += 1
        if k % 50 == 0 and hasattr(u, "_phase"):  # drive env has no arm phase clock
            ph = u._phase()
            print(f"[{label}] step {k} t {time.time() - t0:.0f} s phases {torch.bincount(ph, minlength=7).tolist()} "
                  f"done {len(u.stats['success'])} in_hand {u.in_hand.sum().item()} t_end {u.t_ph[:, 5].max().item():.2f} t {u.t.max().item():.2f}", flush=True)
    print(f"[{label}] {time.time() - t0:.0f} s")
    report(env, label)


def main():
    env = make_env()
    u = env.unwrapped
    if args.mode == "zero":
        rollout(env, lambda obs: torch.zeros(u.num_envs, u.cfg.action_space, device=u.device), "zero")
    elif args.mode == "eval":
        policy = torch.jit.load(args.policy, map_location=u.device)
        policy.eval()
        rollout(env, lambda obs: policy(obs["policy"][:, :args.obs_dim]), "policy")
    else:
        agent = PlacePPOCfg()
        agent.max_iterations = args.iters
        agent.seed = args.seed
        agent.experiment_name = f"bee_{args.task}_residual"
        if args.task in ("load", "unload", "reach", "drive"):  # new skill envs: std in log space (scalar std hit 0.00 on dock)
            agent.actor.distribution_cfg.std_type = "log"
            agent.algorithm.entropy_coef = 0.005
        agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
        log_dir = LOG_DIR / args.run / time.strftime("%Y-%m-%d_%H-%M-%S")
        wenv = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
        runner = OnPolicyRunner(wenv, agent.to_dict(), log_dir=str(log_dir), device=u.device)
        if args.resume:
            runner.load(args.resume)
        runner.learn(num_learning_iterations=agent.max_iterations, init_at_random_ep_len=False)
        out = POLICY_DIR
        runner.export_policy_to_jit(path=str(log_dir / "exported"), filename="policy.pt")
        (out / args.out).parent.mkdir(parents=True, exist_ok=True)  # --out v02/x.pt -> policies/v02/
        shutil.copy(log_dir / "exported" / "policy.pt", out / args.out)
        # reference ONNX copy (weights may go to a side .onnx.data) — the robot file comes from policies/export_onnx.py
        onnx_rel = Path(args.out).with_suffix(".onnx")
        runner.export_policy_to_onnx(path=str(log_dir / "exported"), filename=onnx_rel.name)
        shutil.copy(log_dir / "exported" / onnx_rel.name, out / onnx_rel)
        print(f"[train] exported {out / args.out}, {out / onnx_rel} (log {log_dir})")
        report(env, "train-all")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
