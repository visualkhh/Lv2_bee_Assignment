"""ChoiceItem — (key, label) 고르기 항목 (MenuBar에 들어가는 Component)."""
from __future__ import annotations

from ...component import Component


class ChoiceItem(Component):
    """(key, label) 고르기 항목 — activate()가 key를 돌려준다 (ui.select·관절 고르기 같은 단순 선택)."""

    focusable = True

    def __init__(self, key: str, text: str):
        self.key, self._text = key, text

    def label(self) -> str:
        return self._text

    def activate(self):
        return self.key
