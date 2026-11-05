"""Bee-Pip-Residual: place_pip with the real dice layouts (pips 1..6) and neighbour pucks already on the box.

Same episode as place_env.PlaceEnv (puck in the hand above a near / middle row cell -> down -> release -> up), plus:
- each episode draws a dice face (1..6) in a random grid symmetry that contains the target cell; every other cell of
  that face already holds a puck with probability 0.5 (any placing order; far row cells are filled from the other box
  face, mission M2), each set down with the place error (+-4 mm) and a random yaw.
- the policy also sees which of the 8 cells around the target are occupied; the critic sees how far the neighbours moved.
- success also needs every neighbour to stay put (moved <= 3 mm, tilt <= 5 deg, report 4) and a step penalty is paid
  for neighbour motion.
"""

from __future__ import annotations

import math

import gymnasium as gym
import torch

import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.utils import configclass

import bee_common as bc
import place_env as pe

FACES = {  # (row, col) on the 3 x 3 grid, row 0 = near the robot
    1: [(1, 1)],
    2: [(0, 0), (2, 2)],
    3: [(0, 0), (1, 1), (2, 2)],
    4: [(0, 0), (0, 2), (2, 0), (2, 2)],
    5: [(0, 0), (0, 2), (1, 1), (2, 0), (2, 2)],
    6: [(0, 0), (1, 0), (2, 0), (0, 2), (1, 2), (2, 2)],
}
N_NB = 5
RING = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)]


def _variants():
    """All distinct faces under the 8 grid symmetries, as cell sets."""
    syms = [lambda r, c: (r, c), lambda r, c: (c, 2 - r), lambda r, c: (2 - r, 2 - c), lambda r, c: (2 - c, r)]
    out = set()
    for cells in FACES.values():
        for s in syms:
            for flip in (False, True):
                out.add(frozenset(s(r, 2 - c if flip else c) for r, c in cells))
    return [sorted(v) for v in out]


@configclass
class PipSceneCfg(pe.PlaceSceneCfg):
    nb0 = pe.PlaceSceneCfg().puck.replace(prim_path="{ENV_REGEX_NS}/nb0", init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 0.001)))
    nb1 = pe.PlaceSceneCfg().puck.replace(prim_path="{ENV_REGEX_NS}/nb1", init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.2, 0.001)))
    nb2 = pe.PlaceSceneCfg().puck.replace(prim_path="{ENV_REGEX_NS}/nb2", init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.4, 0.001)))
    nb3 = pe.PlaceSceneCfg().puck.replace(prim_path="{ENV_REGEX_NS}/nb3", init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.6, 0.001)))
    nb4 = pe.PlaceSceneCfg().puck.replace(prim_path="{ENV_REGEX_NS}/nb4", init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.8, 0.001)))


@configclass
class PipEnvCfg(pe.PlaceEnvCfg):
    scene: PipSceneCfg = PipSceneCfg(num_envs=256, env_spacing=1.0)
    observation_space = pe.OBS_DIM + 5 + 8
    nb_prob = 0.5
    nb_pos_m = 0.004  # neighbour set-down error (place p90 ~4 mm)
    w_nb = 50.0


class PipEnv(pe.PlaceEnv):
    cfg: PipEnvCfg

    def __init__(self, cfg: PipEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.nbs = [self.scene[f"nb{k}"] for k in range(N_NB)]
        # for every placeable target (near / middle row cell) the face variants that contain it, padded to N_NB others
        var = _variants()
        self.var_by_cell = []
        for idx in range(len(pe.CELLS)):
            rc = divmod(idx, 3)
            others = [[c for c in v if c != rc] for v in var if rc in v]
            self.var_by_cell.append(torch.tensor([o + [(-9, -9)] * (N_NB - len(o)) for o in others], device=dev))
        self.nb_on = torch.zeros(n, N_NB, dtype=torch.bool, device=dev)
        self.nb_rc = torch.full((n, N_NB, 2), -9, dtype=torch.long, device=dev)
        self.nb0 = torch.zeros(n, N_NB, 3, device=dev)
        self.nb_hit = torch.zeros(n, dtype=torch.bool, device=dev)

    def _nb_motion(self):
        """(n, N_NB) moved distance [m] and (n,) any neighbour moved > 3 mm or tilted > 5 deg."""
        pos = torch.stack([p.data.root_pos_w.torch for p in self.nbs], 1)
        d = (pos - self.nb0).norm(dim=2) * self.nb_on
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        tilt = torch.stack([torch.acos(math_utils.quat_apply(p.data.root_quat_w.torch, up)[:, 2].clamp(-1, 1)) for p in self.nbs], 1)
        return d, (self.nb_on & ((d > 0.003) | (tilt > math.radians(5.0)))).any(dim=1)

    def _occupancy(self):
        r, c = self.cell // 3, self.cell % 3
        occ = []
        for dr, dc in RING:
            hit = (self.nb_rc[:, :, 0] == (r + dr)[:, None]) & (self.nb_rc[:, :, 1] == (c + dc)[:, None]) & self.nb_on
            occ.append(hit.any(dim=1).float())
        return torch.stack(occ, 1)

    def _get_observations(self) -> dict:
        o = super()._get_observations()
        occ = self._occupancy()
        d, _ = self._nb_motion()
        pol = torch.cat([o["policy"], occ], 1)
        return {"policy": pol, "critic": torch.cat([pol, o["critic"][:, pe.OBS_DIM:], 100.0 * d.max(dim=1, keepdim=True).values], 1)}

    def _get_rewards(self) -> torch.Tensor:
        r = super()._get_rewards()
        d, hit = self._nb_motion()
        self.nb_hit |= hit
        done = self._phase() >= 6
        lost = done & self.success & self.nb_hit  # placed fine but knocked a neighbour: not a success
        self.success = self.success & ~(done & self.nb_hit)
        return r - self.cfg.w_success * lost - self.cfg.w_nb * d.sum(dim=1)

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)  # target cell, puck in hand, arm
        m, dev, cfg = len(env_ids), self.device, self.cfg
        cells = self.cell[env_ids]
        rc = torch.full((m, N_NB, 2), -9, dtype=torch.long, device=dev)
        for idx in range(len(pe.CELLS)):
            sel = (cells == idx).nonzero()[:, 0]
            if len(sel):
                vs = self.var_by_cell[idx]
                rc[sel] = vs[torch.randint(0, len(vs), (len(sel),), device=dev)]
        on = (rc[:, :, 0] >= 0) & (torch.rand(m, N_NB, device=dev) < cfg.nb_prob)
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)
        for k, p in enumerate(self.nbs):
            x = pe.BOX_CX + bc.CELL_PITCH * (rc[:, k, 0].float() - 1) + cfg.nb_pos_m * (torch.rand(m, device=dev) * 2 - 1)
            y = bc.CELL_PITCH * (rc[:, k, 1].float() - 1) + cfg.nb_pos_m * (torch.rand(m, device=dev) * 2 - 1)
            pos = torch.stack([x, y, zero + bc.BOX_H + 0.001], 1)
            pos = torch.where(on[:, k, None], pos, torch.tensor([1.0, 0.2 * k, 0.001], device=dev).expand(m, 3)) + org
            quat = math_utils.quat_from_euler_xyz(zero, zero, math.pi * (torch.rand(m, device=dev) * 2 - 1))
            p.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], 1), env_ids=env_ids)
            p.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
            self.nb0[env_ids, k] = pos
        self.nb_on[env_ids], self.nb_rc[env_ids] = on, rc
        self.nb_hit[env_ids] = False


gym.register(
    id="Bee-Pip-Residual",
    entry_point=f"{__name__}:PipEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:PipEnvCfg"},
)
