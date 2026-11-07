"""창(모달) — 전부 Dialog(dialog.py)를 상속한 클래스, 한 파일에 한 클래스.

  Dialog            dialog.py             최상위 부모 — 박스 그리기·크기·지우기·키 루프(run)
  InputDialog       input_dialog.py       한 줄 입력 · validate
  KeyDialog         key_dialog.py         키 하나 (확인창 y/n)
  PagerDialog       pager_dialog.py       결과창 스크롤
  BusyDialog        busy_dialog.py        처리 중 창 (키 루프 없음)
  TeleopDialog      teleop_dialog.py      텔레오퍼
  RouteRvizDialog   route_rviz_dialog.py  A/B 경로 — RViz goal 찍기

쓰는 곳: XxxDialog(ui, ...).run() — 결과를 돌려받아 다음 동작을 정한다. 하위 클래스가 정하는 것은 dialog.py 참고.
"""
from .busy_dialog import BusyDialog
from .dialog import CONTINUE, Dialog
from .input_dialog import InputDialog
from .key_dialog import KeyDialog
from .pager_dialog import PagerDialog
from .route_rviz_dialog import RVIZ_GOAL_TOPIC, RouteRvizDialog, fmt_pose
from .teleop_dialog import TeleopDialog

__all__ = ['BusyDialog', 'CONTINUE', 'Dialog', 'InputDialog', 'KeyDialog', 'PagerDialog',
           'RVIZ_GOAL_TOPIC', 'RouteRvizDialog', 'TeleopDialog', 'fmt_pose']
