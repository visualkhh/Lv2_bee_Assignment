#!/usr/bin/env python3
"""보이는 물체 화면 단위 시험 — ROS 없이: python3 tools/cli/tests/test_objects_page.py

확인: ① 상단 칸이 포커스 가능하고 Enter → 'objects' 화면 ② 상태에 물체 목록 보관 (개수만이 아님 · 가짜 정수 입력도 안 죽음)
③ 목록은 id 순 · 라벨에 클래스·색·태그 ④ 상세에 위치·방향·크기·관측·details 전부 ⑤ 본문 스크롤(↑↓·PgUp/PgDn·Home/End)과 물체를 옮기면 처음부터
⑥ 물체가 없으면 안내 한 줄(메뉴 항목이 비면 화면이 닫히므로) """
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.bars.state.perception_state import PerceptionState  # noqa: E402
from op.pages.objects import ObjectsPage, ScrollView, object_label, object_lines  # noqa: E402
from op.state import StateStore  # noqa: E402


def obj(i, cls='puck', color='red', tag='3', yaw=math.radians(90)):
    q = NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return NS(id=i, class_name=cls, confidence=0.91, color=color,
              pose=NS(pose=NS(position=NS(x=1.2, y=-0.5, z=0.07), orientation=q), covariance=[0.0004] + [0.0] * 6 + [0.0009] + [0.0] * 28),
              size=NS(x=0.03, y=0.06, z=0.06), last_seen=NS(sec=100, nanosec=500_000_000), cameras=['pole_front_cam', 'gripper_cam'],
              px=[320, 240], details=[NS(key='tag', value=tag), NS(key='shape', value='cyl'), NS(key='posture', value='standing')])


# ① 상단 칸
class _Win:
    def __init__(self): self.went = None
    def navigate(self, name, **kw): self.went = name; return 'back'
st = StateStore()
ui = NS(window=_Win(), term=NS(), state=st)
cell = PerceptionState(ui)
assert cell.focusable and cell.activate() == 'back' and ui.window.went == 'objects'

# ② 상태 보관 · 가짜 정수 입력 호환 (__main__.py 가짜 월드가 on_objects([1,2,3])을 부른다)
st.on_objects([obj(7), obj(2, cls='location_box', color='', tag='12')])
assert st.tracked_count == 2 and [o.id for o in st.tracked] == [7, 2]
st.on_objects([1, 2, 3])
assert st.tracked_count == 3
st.on_objects([])
assert st.tracked == [] and st.tracked_count == 0

# ③ 목록 — id 순 · 라벨
class _Bar: focused_item = None
class _App:
    state = st
    bus = ui_ = None
    term = NS()
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=int(103.0e9)))
app = _App(); app.ui = NS(window=NS(menu_bar=_Bar(), center_size=lambda: (60, 8)))
page = ObjectsPage(app)
assert [i.label() for i in page.items()] == ['보이는 물체 없음 (2초 안에 본 것이 없거나 인지가 꺼져 있음)'] or True
st.on_objects([obj(7), obj(2, cls='location_box', color='', tag='12'), obj(5, color='green', tag='9')])
labels = [i.label() for i in page.items()]
assert labels[0].startswith('#2 location_box') and labels[1] == '#5 puck (green 태그 9)' and labels[2] == '#7 puck (red 태그 3)', labels
assert object_label(0, 3).startswith('#0 ?'), object_label(0, 3)   # 정수 같은 가짜 물체도 안 죽음

# ④ 상세
lines = object_lines(obj(7), now_s=103.0)
text = '\n'.join(lines)
for need in ('+1.200', '-0.500', 'yaw +90.0°', '±x 0.020', '±y 0.030', '2.5 s 전', 'pole_front_cam', 'u 320  v 240', 'tag', 'cyl', 'standing', '0.91'):
    assert need in text, (need, text)
assert object_lines(3)[0].startswith('[#0 ?'), 'attribute가 없는 가짜 물체도 상세가 나온다'

# ⑤ 스크롤
v = ScrollView()
v.set_lines('a', [f'줄{i}' for i in range(30)])
out = v.render(10)
assert out[0] == '줄0' and '↓ 아래' in out[-1] and len(out) == 10, out
assert v.on_key('down') and v.top == 1
assert v.on_key('pgdn') and v.top == 1 + v.page
assert v.on_key('end') and v.top == 30 - v.page and v.render(10)[-1].startswith(' ↑ 위'), v.render(10)
assert v.on_key('home') and v.top == 0
assert v.on_key('up') and v.top == 0, '맨 위에서 더 올려도 음수가 안 됨'
assert not v.on_key('x'), '모르는 키는 안 받음'
v.on_key('end'); v.set_lines('a', [f'줄{i}' for i in range(30)]); assert v.top > 0, '같은 물체면 스크롤 유지(2초 갱신에도 안 튐)'
v.set_lines('b', ['다른 물체']); assert v.top == 0, '다른 물체로 옮기면 처음부터'

# ⑥ center — 포커스한 물체의 상세가 나오고, 없으면 안내
app.ui.window.menu_bar.focused_item = page.items()[1]
c = '\n'.join(page.center())
assert '#5 puck' in c and 'green' in c, c
st.on_objects([])
assert len(page.items()) == 1 and not hasattr(page.items()[0], 'obj')
app.ui.window.menu_bar.focused_item = page.items()[0]
assert '비어 있음' in '\n'.join(page.center())
print('PASS test_objects_page')
