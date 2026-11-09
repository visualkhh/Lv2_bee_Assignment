"""ArmView — 팔 관절을 막대로 보여 주고 키로 목표를 고르는 포커스 가능한 부품 (Component).

관절마다 한 줄: 한계 범위 막대 위에 현재 각(●)과 목표 각(◆). ↑↓ 관절 선택 · ←→ 목표 ∓2° · , . ∓1° (미세) ·
z 목표를 현재값으로. **목표만 바꾼다 — 팔은 움직이지 않는다.** 실제 이동은 밖(페이지)이 Enter(extra_key)로 확인을 거쳐 보낸다.
목표는 한계 범위(ARM_LIMITS) 밖으로 못 간다.
"""
from __future__ import annotations

import math

from ..view import View

STEP, FINE = math.radians(2.0), math.radians(1.0)   # ←→ 한 번 · , . 한 번
TRACK, CUR, TGT = '─', '●', '◆'
RED, CYAN, YELLOW, REV, DIM, RESET = '\033[31m', '\033[36m', '\033[33m', '\033[7m', '\033[2m', '\033[0m'


def deg(rad: float) -> str:
    return f'{math.degrees(rad):+.1f}°'


class ArmView(View):
    """joints = 관절 이름들 · limits = {이름: (lower, upper) rad} · current_fn(이름) → 현재 각 rad 또는 None."""

    focusable = True
    hints = '↑↓ 관절 · ←→ 목표 ∓2° · , . ∓1° · z 현재값으로 · Enter 적용(확인 거침)'

    def __init__(self, joints, limits, current_fn, color: bool = True, extra_key=None):
        super().__init__(extra_key)   # 모르는 키(Enter 적용 등)를 페이지로 넘긴다
        self.joints, self.limits, self.current_fn, self.color = tuple(joints), limits, current_fn, color
        self.targets: dict[str, float] = {}   # 바꾼 관절만 (없으면 목표 = 현재)
        self.sel = 0

    # ---------------------------------------------------------------- 값
    def current(self, name: str) -> float | None:
        return self.current_fn(name)

    def target(self, name: str) -> float | None:
        return self.targets.get(name, self.current(name))

    def changes(self, eps: float = math.radians(0.05)) -> dict[str, tuple[float, float]]:
        """목표가 현재와 다른 관절 {이름: (현재, 목표)}."""
        out = {}
        for n, t in self.targets.items():
            cur = self.current(n)
            if cur is not None and abs(t - cur) > eps:
                out[n] = (cur, t)
        return out

    def nudge(self, name: str, delta: float) -> bool:
        """목표를 delta만큼 — 한계 안으로 자른다. 현재 각을 모르면 False."""
        base = self.target(name)
        if base is None:
            return False
        lo, hi = self.limits[name]
        self.targets[name] = max(lo, min(hi, base + delta))
        return True

    def reset(self) -> None:
        self.targets.clear()

    # ---------------------------------------------------------------- 키
    def on_key(self, k) -> bool:
        name = self.joints[self.sel]
        if k == 'up':
            self.sel = (self.sel - 1) % len(self.joints)
        elif k == 'down':
            self.sel = (self.sel + 1) % len(self.joints)
        elif k in ('left', 'right'):
            self.nudge(name, STEP if k == 'right' else -STEP)
        elif k in (',', '.'):
            self.nudge(name, FINE if k == '.' else -FINE)
        elif k in ('z', 'Z'):
            self.reset()
        else:
            return self.forward_extra(k)
        return True

    # ---------------------------------------------------------------- 그리기
    def _bar(self, name: str, width: int) -> str:
        lo, hi = self.limits[name]

        def pos(v):
            return max(0, min(width - 1, round((v - lo) / (hi - lo) * (width - 1))))
        cells = [TRACK] * width
        cur, tgt = self.current(name), self.target(name)
        paint = (lambda ch, c: f'{c}{ch}{RESET}') if self.color else (lambda ch, c: ch)
        if cur is not None and tgt is not None and abs(tgt - cur) > 1e-9:
            cells[pos(tgt)] = paint(TGT, CYAN)
        if cur is not None:
            cells[pos(cur)] = paint(CUR, RED)
        return ''.join(cells)

    def render(self, cols: int) -> list[str]:
        """관절마다 두 줄 — ① 이름 · 현재 → 목표 · Δ  ② 한계 사이 막대. 좁은 터미널(40칸)에서도 안 잘리게 짧게."""
        bar_w = max(10, cols - 14)
        out = []
        for i, n in enumerate(self.joints):
            lo, hi = self.limits[n]
            cur, tgt = self.current(n), self.target(n)
            changed = n in self.targets and cur is not None and abs(self.targets[n] - cur) > 1e-9
            now = deg(cur) if cur is not None else '(모름)'
            goal = deg(tgt) if tgt is not None else '(모름)'
            line = f'{"❯" if i == self.sel else " "} {n} {now}→{goal}'
            if changed:
                line += f' Δ{deg(tgt - cur)}'
            if self.color:
                if i == self.sel:
                    line = REV + line + RESET
                if changed:
                    line = line.replace(f'→{goal}', f'→{YELLOW}{goal}{RESET}' + (REV if i == self.sel else ''), 1)
            out.append(line)
            out.append(f'  {math.degrees(lo):+4.0f}° {self._bar(n, bar_w)} {math.degrees(hi):+4.0f}°')
        return out
