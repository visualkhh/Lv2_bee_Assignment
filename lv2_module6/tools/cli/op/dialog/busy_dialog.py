"""BusyDialog — Dialog 하위 클래스."""
from __future__ import annotations

from ..terminal import DIM
from .dialog import Dialog

class BusyDialog(Dialog):
    """'처리 중…' 창 — 키 루프 없이 떠 있고(run 안 씀), 메뉴 항목이 끝나면 사라진다.
    step(): 단계 줄 (예: '1/2 다시 뜨기 기다림') · progress(): 맨 아래 줄 (지금 기다리는 것 · 회전 표시)."""

    def __init__(self, ui, label: str):
        super().__init__(ui, '처리 중…', [label])
        self.step_txt = ''
        self.progress_txt = ''

    def content(self):
        return [self.title, '', self.body[0], self.ui.term.paint('요청을 보냈습니다. 결과가 나오면 창이 바뀝니다.', DIM),
                self.step_txt, self.progress_txt]

    def step(self, text: str) -> None:
        self.step_txt = text
        self.progress_txt = ''
        self.draw()

    def progress(self, text: str) -> None:
        self.progress_txt = text
        self.draw()
