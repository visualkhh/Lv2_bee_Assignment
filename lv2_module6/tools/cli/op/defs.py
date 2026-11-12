"""공유 상수 — 메뉴 내비게이션 반환값."""
from __future__ import annotations

BACK = 'back'
MAIN = 'main'
QUIT = 'quit'
STAY = 'stay'
TIMEOUT = '__timeout__'  # 입력 대기 시간 초과 → 화면만 다시 그림 (에러 아님)

# 메뉴 입력 대기 [s] — 이 시간 동안 입력이 없으면 헤더 상태바를 다시 그린다.
MENU_REFRESH_S = 2.0
