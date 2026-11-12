"""상단 상태바 — State(칸) 여러 개를 StateBar가 줄로 그린다. 한 파일에 한 클래스.

  State            state.py               칸 하나의 최상위 부모 (Component) — text() (빈 문자열 = 안 보임) · focusable · activate()
  StateBar         state_bar.py           State를 줄(row)로 받아 그림 · add()로 칸·줄 추가 · ←→ 포커스 칸 이동
  TopStateBar      top_state_bar.py       화면 맨 위 기본 구성 (아래 State들)
  OperationState · MapState · EstopState · DeviceState · ConnectionState · PerceptionState · TaskState ·
  DiagState · TopicRateState · RobotState(모터 줄) · PowerState(전원 줄) · NoDataState

새 칸: State를 상속해 text()를 만들고 ui.window.top_bar.add(XxxState(ui), row=…) 또는 TopStateBar 구성에 넣는다.
Tab으로 고를 수 있게: focusable = True + activate() (예: DiagState → window.navigate('diagnostics')).
"""
from .state import State
from .state_bar import StateBar
from .top_state_bar import TopStateBar

__all__ = ['State', 'StateBar', 'TopStateBar']
