#!/usr/bin/env python3
"""학습 정책(TorchScript .pt) → ml 패키지 정책 파일(.policy, include/ml/mlp_policy.hpp가 읽음).

    python3 export_policy.py policies/place_residual_sag.pt ../model/skills/place_cell/place_cell.policy   # 폴더마다 meta.yaml도 (skills/include/skills/model.hpp)
    python3 export_policy.py plan_next.pt ../model/plan/plan_next.policy --act tanh

rsl_rl 정책 (obs_normalizer + mlp: Linear·ELU…) 과 sb3 정책 (Linear·Tanh…, 정규화 없음) 둘 다.
층은 state_dict의 2차원 weight 순서대로. 끝에 무작위 입력 하나와 TorchScript 출력값을 'test'로 넣어 C++이 불러올 때 검산.
"""
import argparse

import torch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pt")
    ap.add_argument("out")
    ap.add_argument("--act", choices=["elu", "tanh"], default="elu")
    a = ap.parse_args()
    m = torch.jit.load(a.pt, map_location="cpu").eval()
    sd = m.state_dict()
    ws = [k for k, v in sd.items() if k.endswith("weight") and v.dim() == 2]
    obs_dim = sd[ws[0]].shape[1]
    if "obs_normalizer._mean" in sd:
        eps = float(getattr(m.obs_normalizer, "eps", 0.01))
        mean = sd["obs_normalizer._mean"].reshape(-1)
        scale = 1.0 / (sd["obs_normalizer._std"].reshape(-1) + eps)
    else:
        mean, scale = torch.zeros(obs_dim), torch.ones(obs_dim)
    act_dim = sd[ws[-1]].shape[0]
    fmt = lambda t: " ".join(f"{float(v):.9g}" for v in t.reshape(-1))
    lines = [f"bee_policy 1 {a.act} {obs_dim} {act_dim} {len(ws)}", fmt(mean), fmt(scale)]
    for k in ws:
        w, b = sd[k], sd[k[: -len("weight")] + "bias"]
        lines += [f"{w.shape[0]} {w.shape[1]}", fmt(w), fmt(b)]
    torch.manual_seed(0)
    x = torch.randn(1, obs_dim)
    with torch.no_grad():
        y = m(x).reshape(-1)
    lines.append("test " + fmt(x) + " " + fmt(y))
    with open(a.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{a.out}: obs {obs_dim} act {act_dim} layers {len(ws)} ({a.act})")


if __name__ == "__main__":
    main()
