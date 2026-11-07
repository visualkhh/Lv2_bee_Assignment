"""Component — 화면 부품의 최상위 부모 (메뉴 항목 · 상태 칸 · 메뉴 막대 · 상태 막대)."""
from __future__ import annotations


class Component:
    """화면에 보이고, 키보드로 포커스할 수 있고(focusable), 고르면 동작하는(activate) 부품.

      focusable   True면 Window가 키보드 포커스를 줄 수 있다
      on_key(k)   포커스된 동안 받은 키 하나 → 처리했으면 True (Window가 다시 그림)
      activate()  Enter로 골랐을 때 · 또는 소스에서 직접 불렀을 때 할 동작 →
                  결과 STAY/BACK/MAIN/QUIT(화면 이동) 또는 값 (Window.run(once=True)가 그대로 돌려줌)
    """

    focusable = False

    def on_key(self, k) -> bool:
        return False

    def activate(self):
        return None
