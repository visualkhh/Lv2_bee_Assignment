"""plan_next 학습 — 가상 미션 + 막힌 행동을 가리는 PPO (sb3-contrib MaskablePPO). CPU로 충분.

    python3 train_plan.py train --steps 300000 --out plan_next.zip   # 기준선 따라 하기 → 강화학습
    python3 train_plan.py eval --model plan_next.zip --episodes 1000   # 기준선(지금 규칙)과 비교
    python3 train_plan.py eval --episodes 1000                         # 기준선·계산기만
    python3 train_plan.py train --teacher solver                       # 계산기를 따라 한 뒤 강화학습
"""
import argparse
import statistics

import numpy as np

from baseline import baseline_action
from mission_sim import MissionSim
from solver import solver_action

TEACHERS = {"baseline": baseline_action, "solver": solver_action}


def run(env, choose, episodes, seed):
    scores, times = [], []
    for k in range(episodes):
        env.reset(seed=seed + k)
        done = False
        while not done:
            _, _, term, trunc, info = env.step(choose(env))
            done = term or trunc
        scores.append(info["score"])
        times.append(info["time"])
    full = sum(s >= 40 for s in scores) / episodes
    return {"점수 평균": round(statistics.mean(scores), 2), "만점(40) 비율": round(full, 3),
            "시간 평균 s": round(statistics.mean(times))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["train", "eval"])
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--out", default="plan_next.zip")
    ap.add_argument("--model", default="")
    ap.add_argument("--episodes", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=10_000)  # 평가 문제는 학습과 다른 시드
    ap.add_argument("--lr", type=float, default=1e-4, help="강화학습 학습률 (따라 하기 뒤에는 작게)")
    ap.add_argument("--bc_episodes", type=int, default=500, help="따라 하기 판 수 (0 = 강화학습만)")
    ap.add_argument("--teacher", choices=TEACHERS, default="baseline", help="따라 할 플래너: 지금 규칙 · 계산기")
    a = ap.parse_args()

    from sb3_contrib import MaskablePPO
    if a.mode == "train":
        model = MaskablePPO("MlpPolicy", MissionSim(seed=0), n_steps=4096, batch_size=512, gamma=0.995,
                            ent_coef=0.001, learning_rate=a.lr, verbose=0, seed=0)
        if a.bc_episodes:
            warm_start(model, a.bc_episodes, TEACHERS[a.teacher])
            print(f"{a.teacher} 따라 하기 뒤:", run(MissionSim(), policy_of(model), 300, a.seed))
        model.learn(total_timesteps=a.steps)
        model.save(a.out)
        a.model = a.out
    env = MissionSim()
    print("기준선 (지금 규칙):", run(env, baseline_action, a.episodes, a.seed))
    print("계산기 (최단 순서):", run(env, solver_action, a.episodes, a.seed))
    if a.model:
        print("plan_next (학습):  ", run(env, policy_of(MaskablePPO.load(a.model)), a.episodes, a.seed))


def policy_of(model):
    return lambda e: int(model.predict(e._obs(), action_masks=e.action_masks(), deterministic=True)[0])


def warm_start(model, episodes, teacher, epochs=30):
    """teacher(지금 규칙 또는 계산기)를 먼저 따라 하게 함 (행동 복제) — 처음부터 강화학습만 하면 '바로 복귀'에 갇힘."""
    import torch
    env = MissionSim(seed=1)
    obs, masks, acts = [], [], []
    for k in range(episodes):
        env.reset(seed=1 + k)
        done = False
        while not done:
            act = teacher(env)
            obs.append(env._obs()); masks.append(env.action_masks()); acts.append(act)
            _, _, term, trunc, _ = env.step(act)
            done = term or trunc
    pol = model.policy
    o = torch.as_tensor(np.array(obs), device=pol.device)
    m = np.array(masks)
    act = torch.as_tensor(np.array(acts), device=pol.device)
    opt = torch.optim.Adam(pol.parameters(), lr=1e-3)
    for _ in range(epochs):
        for idx in torch.randperm(len(o)).split(512):
            dist = pol.get_distribution(o[idx], action_masks=m[idx.cpu().numpy()])
            loss = -dist.log_prob(act[idx]).mean()
            opt.zero_grad(); loss.backward(); opt.step()
    print(f"따라 하기: 예시 {len(o)}개, 마지막 손실 {loss.item():.3f}")


if __name__ == "__main__":
    np.set_printoptions(precision=3)
    main()
