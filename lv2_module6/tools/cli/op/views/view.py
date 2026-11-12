"""View — 지도·팔처럼 포커스 가능한 화면 부품의 공통 부모 (Component).

  render(*args)   화면에 그릴 줄 목록
  extra_key(k)    뷰가 모르는 키(Enter 적용 등)를 페이지로 넘기는 콜백
  forward_extra   그 콜백을 안전하게 호출 (없으면 False)
"""
from __future__ import annotations

from ..component import Component


class View(Component):
    focusable = True
    hints = ''

    def __init__(self, extra_key=None):
        self.extra_key = extra_key

    def forward_extra(self, k) -> bool:
        return bool(self.extra_key and self.extra_key(k))

    def render(self, *args) -> list[str]:
        raise NotImplementedError
