"""Bar — 메뉴 막대·상태 막대의 공통 부모 (Component).

  lines(focused=…)  화면에 그릴 줄 목록 (하위 클래스가 구현)
  idx               지금 포커스한 줄/칸 번호
"""
from __future__ import annotations

from ..component import Component


class Bar(Component):
    focusable = True

    def __init__(self):
        self.idx = 0

    def lines(self, focused: bool = True) -> list[str]:
        raise NotImplementedError
