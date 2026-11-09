"""Menu — 방향키 메뉴. items()의 MenuItem들을 보여주고 고른 항목의 run()을 실행한다.

흐름 규약 (모든 메뉴 공통):
  - 항목 run()은 BACK/MAIN/QUIT/STAY만 돌려준다. 화면 이동은 반환값으로만.
    BACK은 그 항목이 연 하위 화면이 닫혔다는 뜻 — 부모 메뉴는 그대로 남는다.
  - 실행 중 print()는 전부 Window.perform이 가로채 결과창(pager)에 보여준다 — 다이얼로그보다 먼저
    알려야 할 경고·맥락은 print가 아니라 confirm(body=...)/modal_input(body=...)에 넣는다.
  - 취소(Esc·아니오)는 아무 출력 없이 STAY.
  - 누르면 바로 '처리 중…' 창 (MenuItem.shows_busy, 기본 True) — ROS 대기로 몇 초 걸려도 눌린 걸 알게.
    하위 화면을 여는 항목·전체 화면을 쓰는 항목(텔레오퍼)은 shows_busy = False.
  - 몇 초 넘게 기다리는 명령은 그동안 ui.progress('무엇을 … N/M초')로 기다리는 대상을 보여준다.
    여러 단계면 ui.step('1/2 …'). 명령줄(run-cli <명령>)에서는 둘 다 무시돼 print만 남는다.
  - 창(확인·입력·결과)이 떠 있는 동안에도 상단 상태바는 1초마다 갱신된다 (ui.tick).
"""
from __future__ import annotations

from ..pages.page import Page
from .item import MenuItem


class Menu(Page):
    """방향키 메뉴 — items()의 MenuItem들을 보여주고 고른 항목의 run()을 실행."""

    #: 최상위 메뉴 — 뒤로·메인 없음
    root = False

    def items(self) -> list[MenuItem]:
        return []

    def center(self) -> list[str]:
        return []

    def show(self) -> str:
        """Window.run — 진입하면 첫 항목에 포커스 · 2초마다 항목·중앙 다시 계산 · MAIN/QUIT/BACK만 돌려준다.
        하위 화면의 BACK은 '그 화면을 닫음'이라 여기는 그대로 남는다 (두 단계 뒤로 가지 않게)."""
        return self.ui.window.run(self.title, lambda: [i for i in self.items() if i.visible()],
                                  self.center, back=not self.root, main=not self.root)
