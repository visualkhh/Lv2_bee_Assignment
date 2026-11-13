#!/usr/bin/env python3
"""현장 디버그 (op/fielddebug.py · 미션 화면 새 줄) 단위 시험 — ROS 없이: python3 tools/cli/tests/test_field_debug.py"""
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op import fielddebug as fd  # noqa: E402
from op.pages.mission import mission_lines  # noqa: E402
from op.state import StateStore  # noqa: E402
from op.terminal import Terminal  # noqa: E402
from ros.msgs import _Msg  # noqa: E402

SEQ = 'R G R R G R G G Y G Y G'.split()
COLOR = {'R': 'red', 'Y': 'yellow', 'G': 'green'}


def run_cycle(tr, t0, seq, hz=10.0, glitch=False):
    """발제 신호 — 한 번 = 1초 켜짐 + 1초 꺼짐, 12번 뒤 3초 꺼짐. glitch면 켜짐 중 한 프레임 놓침·다른 색 한 프레임."""
    t = t0
    for c in seq:
        for i in range(int(hz)):
            state = COLOR[c]
            if glitch and i == 4:
                state = 'off'          # min_off_s보다 짧은 꺼짐 — 같은 점멸
            if glitch and i == 6:
                state = 'yellow' if c != 'Y' else 'red'   # 다수결로 무시
            tr.feed(t, state)
            t += 1 / hz
        for _ in range(int(hz)):
            tr.feed(t, 'off')
            t += 1 / hz
    for _ in range(int(2 * hz)):   # 1초 꺼짐에 이어 2초 더 = 3초
        tr.feed(t, '')
        t += 1 / hz
    return t


# 1) 점멸 세기 — 깨끗한 신호 · 흔들리는 신호 둘 다 12번 그대로
for glitch in (False, True):
    tr = fd.SignalTracker()
    t = 0.0
    for _ in range(5):                  # 시작 전 꺼짐
        tr.feed(t, 'off'); t += 0.1
    for c in SEQ[:5]:                   # 사이클 중간 — 5번째까지
        for _ in range(10):
            tr.feed(t, COLOR[c]); t += 0.1
        for _ in range(10):
            tr.feed(t, 'off'); t += 0.1
    assert len(tr.seg) == 5 and '5/12 (R3 Y0 G2)' in tr.summary(), tr.summary()
    assert '꺼짐 ' in tr.summary()
    tr = fd.SignalTracker()
    run_cycle(tr, 0.0, SEQ, glitch=glitch)
    assert tr.last_cycle == SEQ and tr.seg == [], (glitch, tr.last_cycle, tr.seg)

# 2) 짧은 켜짐(잡음)은 점멸이 아님 · 잡음이 3초 꺼짐을 끊지 않음
tr = fd.SignalTracker()
t = run_cycle(tr, 0.0, ['R'] * 3)
assert tr.last_cycle == ['R'] * 3
tr.feed(t, 'green'); tr.feed(t + 0.1, 'off'); tr.feed(t + 0.5, 'off')
assert tr.seg == [], tr.seg

# 3) 다수결 안 되면 ?
tr = fd.SignalTracker()
for i in range(10):
    tr.feed(i * 0.1, ['red', 'green', 'yellow', 'all'][i % 4])
tr.feed(1.5, 'off')
assert tr.seg == ['?'], tr.seg

# 4) 신호등 고르기 — 켜진 것 우선, 오래된 것은 안 씀
assert fd.light_state([(9.9, 'off'), (9.8, 'red')], 10.0) == 'red'
assert fd.light_state([(9.0, 'red'), (9.9, 'off')], 10.0) == 'off'
assert fd.light_state([], 10.0) == ''

# 5) 정답 비교 — 개수(순서 무관)와 순서
ans = fd.parse_answer('R G R R G R G G Y G Y G')
assert fd.answer_match(ans, SEQ).startswith('개수 맞음 · 순서 12/12')
swapped = SEQ[:]; swapped[0], swapped[1] = swapped[1], swapped[0]
r = fd.answer_match(ans, swapped)
assert r.startswith('개수 맞음') and '순서 10/12 (틀린 자리 1,2)' in r, r
assert fd.answer_match(ans, SEQ[:11]).startswith('개수 다름'), fd.answer_match(ans, SEQ[:11])
assert fd.parse_answer('rgr, ygx') == ['R', 'G', 'R', 'Y', 'G']

# 6) 로그 — 노드별 최신 파일 · WARN/ERROR만 시각 순
with tempfile.TemporaryDirectory() as d:
    d = Path(d)
    (d / 'coordinator_1_100.log').write_text('[WARN] [5.0] [coordinator]: old\n')
    time.sleep(0.01)
    (d / 'coordinator_2_200.log').write_text('[INFO] [1.0] [coordinator]: hi\n[ERROR] [3.0] [coordinator]: boom\n')
    (d / 'perception_node_3_300.log').write_text('[WARN] [2.0] [perception]: TF\n')
    (d / 'python3_4_400.log').write_text('[ERROR] [1.0] [x]: no\n')
    names = sorted(p.name for p in fd.newest_logs(d))
    assert names == ['coordinator_2_200.log', 'perception_node_3_300.log'], names
    assert [ln.split(': ')[1] for ln in fd.recent_bad(40, d)] == ['TF', 'boom']

# 7) 부하 · 빌드 — 이 머신에서 돌아가기만 (Mac은 /proc 없어 상위 목록 빈 것)
assert fd.load_ratio() is not None
fd.top_procs(); fd._procs["t"] -= 10; assert isinstance(fd.top_procs(), list)
fd._procs["t"] -= 10; assert all(len(r) == 3 and isinstance(r[1], int) for r in fd.proc_usage())
b = fd.build_info()   # repo 밖으로 복사한 cli면 None
assert b is None or b['commit'], b

# 7b) 스냅샷 영상 — rgb8 (줄 끝 여분 바이트 있음) → JPEG (cv2·numpy 없으면 건너뜀)
try:
    import cv2  # noqa: F401
    with tempfile.TemporaryDirectory() as d:
        w, h, step = 4, 2, 16
        data = bytes(([255, 0, 0] * w + [7] * (step - 3 * w)) * h)    # 빨강
        out = Path(d) / 'cam.jpg'
        assert fd.save_jpeg(_Msg(encoding='rgb8', width=w, height=h, step=step, data=data), out)
        px = cv2.imread(str(out))[0, 0]
        assert px[2] > 200 and px[0] < 60, px                       # BGR로 저장 — R 큼
        assert not fd.save_jpeg(_Msg(encoding='16UC1', width=w, height=h, step=step, data=data), out)
except ImportError:
    print('(cv2 없음 — 영상 저장 시험 건너뜀)')

# 8) 미션 화면 — 데이터 없으면 새 줄은 빠지고, 있으면 나옴
st = StateStore()
text = '\n'.join(mission_lines(st, Terminal(color=False)))
assert '[신호등 지금]' not in text and '[Nav2]' not in text and '[인지]' not in text
assert '[부하]' in text and ('[빌드]' in text) == (b is not None), text

now = time.time()
stamp = _Msg(sec=int(now), nanosec=int((now % 1) * 1e9))
st.on_objects([_Msg(class_name='traffic_light', color='red', last_seen=stamp, cameras=['pole_front_cam'], details=[]),
               _Msg(class_name='barcode', color='', last_seen=stamp, cameras=['pole_front_cam', 'gripper_cam'],
                    details=[_Msg(key='content', value='P2-01')])])
st.signal_answer = 'R G R R G R G G Y G Y G'
st.on_signal_status(_Msg(ok=True, error='', red=4, yellow=2, green=6, cycles=1, raw='G R R R G R G G Y G Y G'))
st.on_diagnostics([_Msg(name='/Beebot/인지/perception: 상태', level=0, message='인지 중', values=[
    _Msg(key='시각 차 초과 TF', value='3'), _Msg(key='오래된 영상 제외', value='1'), _Msg(key='TF 최신 대체', value='7'),
    _Msg(key='바코드 (장소 작업 지시)', value='zxing-cpp (1D) · 읽음: P2-01'),
    _Msg(key='처리 시간 ms 평균/최대 pole_front_cam', value='YOLO 35/60 · AprilTag 3/5 · 전체 80/120')])])
st.nav2_life.update(planner_server='active', controller_server='inactive')
st.values['map_mode'] = 'SLAM'
st.on_mission(_Msg(phase='BARCODE', error='', note='', barcode='', count_r=0, count_y=0, count_g=0, order='',
                   placed=0, total=0, results=[], tray=[]))
text = '\n'.join(mission_lines(st, Terminal(color=False)))
for want in ('[신호등 지금] R 켜짐 · 0/12', '정답 R G R R G R G G Y G Y G · 해독 개수 맞음 · 순서 10/12 (틀린 자리 1,2)',
             '[인지] pole_front_cam YOLO 35/60 전체 80/120ms', '시각 차 초과 TF 3 · 오래된 영상 제외 1 · TF 최신 대체 7',
             '인지 읽음 P2-01', 'gripper_cam가 봄', '[Nav2] planner ✓ · controller inactive · bt 없음'):
    assert want in text, (want, text)
st.mission_cmd = ('start', time.monotonic() - 10)
st.diag_values.clear()
text = '\n'.join(mission_lines(st, Terminal(color=False)))
assert '찾기 창 170s 남음 (손캠만)' in text, text
print('PASS test_field_debug')
