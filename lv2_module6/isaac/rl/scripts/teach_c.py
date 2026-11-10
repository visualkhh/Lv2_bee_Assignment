"""Scripted teacher for Bee-Stand_c-Residual: drives the 3-value hand action along the baseline arc (check of the env and a
baseline of the method C action space). python teach_c.py --num_envs 64 --episodes 256 [--cfg k=v,...] [--policy x.pt: run that TorchScript policy instead]"""
import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--episodes", type=int, default=256)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--cfg", default="")
parser.add_argument("--speed", type=float, default=1.0, help="teacher action magnitude cap")
parser.add_argument("--vh", type=float, default=0.3, help="teacher hand speed along the arc [m/s]")
parser.add_argument("--fb", choices=["xt", "ee"], default="xt")
parser.add_argument("--puck", choices=["box", "cyl"], default="box")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from bee_isaac.tasks import stand_c_env as sc  # noqa: E402,F401


def main():
    cfg = sc.StandCEnvCfg()
    cfg.scene.num_envs, cfg.seed, cfg.sim.device = args.num_envs, args.seed, args.device
    for kv in filter(None, args.cfg.split(",")):
        k, v = kv.split("=")
        cur = getattr(cfg, k)
        setattr(cfg, k, (v.strip().lower() in ("1", "true")) if isinstance(cur, bool) else (tuple(float(x) for x in v.split(":")) if ":" in v else type(cur)(v)))
    if args.puck:
        cfg.scene.puck.spawn.usd_path = [x for x in cfg.scene.puck.spawn.usd_path if f"/{args.puck}/" in x]
    env = gym.make("Bee-Stand_c-Residual", cfg=cfg)
    u = env.unwrapped
    obs, _ = env.reset()
    pol = torch.jit.load(args.policy, map_location=u.device).eval() if args.policy else None
    n = u.num_envs
    W = u.wp.shape[1]
    n_arc = int(round(cfg.arc_end_deg / 7.5)) + 1  # waypoints 1 .. n_arc are the arc
    idx = list(range(1, n_arc + 1)) + [W - 2]  # arc, then the set-down point
    ar = torch.arange(n, device=u.device)
    k = 0
    t0 = time.time()
    sc_n = lambda: len(u.stats["success"]) + int(np.sum(u.stats["dropped"]))
    while sc_n() < args.episodes:
        with torch.inference_mode():
            ph = u.ph
            ee = u._ee_b()[0]
            P = torch.stack([u.arm.fk(u.wp[:, i])[0] for i in idx], 1)  # (n, K, 3)
            seg = (P[:, 1:] - P[:, :-1]).norm(dim=2)
            cum = torch.cat([torch.zeros(n, 1, device=u.device), seg.cumsum(1)], 1)  # (n, K)
            L_arc = cum[:, n_arc - 1]
            tau = u.t_in
            t_arc = L_arc / args.vh
            s_ = torch.where(tau < t_arc, args.vh * tau, torch.where(tau < t_arc + 0.5, L_arc, L_arc + args.vh * (tau - t_arc - 0.5)))
            s_ = torch.minimum(s_, cum[:, -1])
            j = (torch.searchsorted(cum.contiguous(), s_[:, None].contiguous())[:, 0]).clamp(1, len(idx) - 1)
            f = ((s_ - cum[ar, j - 1]) / (cum[ar, j] - cum[ar, j - 1]).clamp(min=1e-6)).clamp(0, 1)
            path = P[ar, j - 1] + f[:, None] * (P[ar, j] - P[ar, j - 1])
            tgt = torch.where((ph == 1)[:, None], u.p_pinch, path)
            a = u._frame(tgt - (u.x_t if args.fb == 'xt' else ee)) / cfg.step_m
            a = a.clamp(-1, 1)
            if k == 0:
                print("pinch point vs FK(wp1) mm:", (1000 * (u.p_pinch - P[:, 0]).abs().max(dim=0).values).tolist(), flush=True)
            if pol is not None:
                a = pol(obs["policy"])
            obs, *_ = env.step(a)
        k += 1
        if k % 100 == 0:
            print(f"step {k} t {time.time() - t0:.0f}s phases {torch.bincount(u.ph, minlength=9).tolist()} done {sc_n()} up_z {u._up_z().mean().item():.2f} ee-xt mm {1000 * (u._ee_b()[0] - u.x_t).norm(dim=1).mean().item():.1f}", flush=True)
    s = u.stats
    ok = np.array(s["success"], dtype=bool)
    print(f"[teacher] episodes {sc_n()}: success {ok.sum()}/{sc_n()} = {100 * ok.sum() / sc_n():.1f} %, dropped {int(np.sum(s['dropped']))}")
    print("early failures: box_moved", int(np.sum(s["box_moved"])), "nb_fell", int(np.sum(s["nb_fell"])), "fell_over(+stuck)", int(np.sum(s["fell_over"])))
    sp = s.get("stuck_ph", [])
    print("stuck in phase:", {p: sp.count(p) for p in sorted(set(sp))})
    if "stuck_up" in s:
        su, sw, sl = (np.array(s[k_]) for k_ in ("stuck_up", "stuck_wn", "stuck_slide"))
        print("stuck: tilt deg p10/50/90", np.percentile(np.degrees(np.arccos(np.clip(su, -1, 1))), [10, 50, 90]).round(0), "| wn p10/50/90", np.percentile(sw, [10, 50, 90]).round(2), "| slide mm p10/50/90", np.percentile(1000 * sl, [10, 50, 90]).round(0))
    for kk in ("up_max", "up_open"):
        v = np.degrees(np.arccos(np.clip(np.array(s[kk]), -1, 1)))
        print(f"{kk} tilt deg p10 {np.percentile(v, 10):.0f} median {np.median(v):.0f} p90 {np.percentile(v, 90):.0f}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
