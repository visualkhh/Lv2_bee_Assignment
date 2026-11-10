# DEVELOP — operator_tool(CLI) 개발자용

CLI를 고치거나 화면·메뉴를 추가하는 개발자용 문서입니다. 쓰는 법(명령·키)은 [../README.md](../README.md)를 봅니다.

- Python 3 · rclpy만 씀 (curses·textual 같은 TUI 라이브러리 없음)
- 화면은 **stdout에 글자 + ANSI escape code**를 쓰고, 키는 **stdin에서 바이트를 직접** 읽어서 만든다
- ROS 없이 시험: `ros/fake/FakeRos.py` (가짜 ROS층) · `ros/docker/DockerRos.py` (도커 debug 파일 브리지) · `tests/` (pty 키 입력 시나리오)

## 1. 구조

```text
cli/
├── operator_tool.py   # OperatorTool(Node): 구독 배선 + argparse + main. 화면 번호 → Page (screen())
├── __main__.py        # 모드 해석(real/fake/docker) → 대역 설치 → operator_tool 실행 (python -m cli)
├── ros/
│   ├── interface.py   # RosInterface·RosNode·install_modules (fake/docker 공통)
│   ├── msgs.py        # 공유 메시지 클래스
│   ├── fake/          # FakeRos.py (프로세스 안 가짜 ROS) + map/
│   └── docker/        # DockerRos.py(파일 브리지) · FakerStack.py(test-faker 자동 시작)
├── tests/             # pty 시나리오 · views/map·views/arm·DockerRos 단위 시험
└── op/
    ├── terminal.py    # Terminal — 크기 · 색 · 표시 너비(fit). ROS·메뉴를 모름
    ├── keys.py        # KeyReader(키 읽기) · set_cbreak — 창·메뉴·poll_key 공용
    ├── component.py   # Component — 화면 부품 최상위 부모 (focusable · on_key · activate)
    ├── window/        # 화면 틀 — Window(조합·그리기·키 루프·신호) · 키 규약
    ├── bars/          # 화면 막대
    │   ├── bar.py     # Bar(Component) — MenuBar·StateBar 공통 부모
    │   ├── menu/      # 메뉴 막대 — MenuBar → LeftSideMenuBar · ChoiceItem
    │   └── state/     # 상단 상태바 — State(칸) 하위 클래스들 + StateBar + TopStateBar
    ├── dialog/        # 창 — Dialog(dialog.py) 최상위 부모 + 하위 클래스 한 파일에 하나 (Input·Key·Pager·Busy·Teleop·RouteRviz)
    ├── views/         # 화면 부품
    │   ├── view.py    # View(Component) — MapView·ArmView 공통 부모
    │   ├── map/       # 터미널 지도 — MapGrid(벽 우선 피라미드) · Viewport(중심·확대) · MapView
    │   └── arm/       # 팔 관절 막대 — ArmView(한계 범위 막대 + 현재 ● 목표 ◆)
    ├── ui.py          # Ui — 터미널 모드 · print 가로채기 · confirm/modal_input/pager/select 진입점 · ui.window
    ├── state.py       # StateStore — 구독한 ROS 상태 보관 (화면은 여기만 읽음)
    ├── bus.py         # Bus — 발행·대기·모드 게이트 · cmd_* (한 번 실행 명령과 메뉴가 같이 씀)
    ├── defs.py        # BACK/MAIN/QUIT/STAY/TIMEOUT · MENU_REFRESH_S
    ├── consts.py      # 관절 이름 · 파라미터 변환
    └── pages/         # Page → Menu → 각 화면 (MenuItem 객체)
        ├── page.py    # Page 베이스만
        ├── mainmenu/ status/ mode/ manual/ arm/ map/ mapmenu/ params/ devices/ topics/  # 화면별 package
        ├── nodes/     # 노드별 추가 상태 (feature.py + <node>.py)
        ├── ab/        # A/B 화면 + core.py + <node>/command.json
        └── calibration/  # 칼리브레이션 화면 + core.py + wheel/
```

`./run-cli status`처럼 인자가 있으면 `Bus.cmd_*`만 돌고 화면은 없습니다. 인자가 없고 TTY면 `MainMenu`가 뜹니다.

## 2. TUI는 어떻게 만들었나

### 2.1 터미널 크기

```python
# op/terminal.py
size = shutil.get_terminal_size()   # (columns, lines) — 내부는 ioctl(TIOCGWINSZ)
```

창 크기가 바뀌면 OS가 `SIGWINCH`를 보내지만 받지 않습니다. **그릴 때마다 크기를 다시 묻고**, 바뀌었으면 전체를 지운 뒤 다시 그립니다 (`Ui.draw`의 `_frame_size`).

### 2.2 레이아웃 유지 — 매번 다시 그리되, 지우지 않는다

지우고 그리면 빈 화면이 잠깐 보여 깜빡입니다. 그래서 커서만 맨 위로 옮기고 같은 자리에 덮어씁니다.

```python
# op/ui.py draw()
if self._frame_size != size:      # 처음 · 창 크기 변경 때만
    self.term.clear()             # \033[2J  화면 전체 지우기
else:
    self.console.write('\033[H')  # 커서를 (1,1)로
for ln in frame:                  # 항상 정확히 rows-1줄 (짧아진 화면의 흔적이 안 남게)
    self.console.write('\033[K' + ln + '\n')   # \033[K  그 줄의 나머지 지우기
```

- 4분할은 문자열 조립입니다: 줄마다 `왼쪽 메뉴(폭 lw) + "| " + 중앙 내용(폭 cw)`. 위는 타이틀·상태바, 아래는 키 안내 한 줄.
- 폭 맞추기는 `Terminal.fit(s, w)`가 합니다. 한글은 화면에서 2칸이라 `len()`이 아니라 **표시 너비**로 자르고 채웁니다 (`unicodedata.east_asian_width`가 W/F면 2칸). 색 코드는 안 보이므로 너비에서 뺍니다. 넘치면 `…`.
- 입력이 없어도 2초(`MENU_REFRESH_S`)마다 다시 그려 상단 상태바가 갱신됩니다.
- 폭 70 미만·높이 18 미만이면 분할 없이 세로로 쌓습니다.

### 2.3 화면 가운데에 입력창 놓기

화면은 **커서 위치를 지정해 쓸 수 있는 글자 격자**입니다.

```python
# op/dialog/dialog.py Dialog.draw()
console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')    # top행 left열로 이동 후 쓰기
for i, s in enumerate(content):
    console.write(f'\033[{top + 1 + i};{left}H│' + fit(s, w - 2) + '│')
```

- 뒤의 메뉴 위에 박스를 덮어 그려서 팝업처럼 보입니다. 닫히면 메뉴를 다시 그려 덮습니다.
- 입력창에 진짜 위젯은 없습니다. 키 하나마다 `buf`에 붙이고 박스를 다시 그려 `값> 0.3▏`처럼 보여 줄 뿐입니다 (`▏`도 글자).
- 창 크기가 바뀌어 박스 위치가 달라지면 예전 자리를 공백으로 지웁니다 (`Ui._erase_rect`).

#### 창은 전부 Dialog를 상속한다 (Page → Menu처럼)

박스 그리기·크기·지우기·키 루프(`run()`)는 기반 `Dialog`가 맡고, 창마다 하위 클래스가 세 가지만 정한다.
밖에서 창의 내부 줄(`lines` 등)을 직접 바꾸지 않는다 — 내용은 `content()`가 그때그때 만든다.

| 메서드 | 정하는 것 |
|---|---|
| `content()` | 박스 안 줄 목록 (기본: 제목 · 빈 줄 · body) |
| `on_key(k)` | 키 하나 → `CONTINUE`(창 유지) 또는 결과(창 닫고 `run()`이 돌려줌) |
| `on_tick()` | 키가 있든 없든 0.1초마다 — ROS 폴링·명령 재전송 등 (기본: 상단 상태바 갱신) |
| `fallback()` | TTY가 아닐 때(명령줄·파이프) `run()`의 결과 |

```text
Dialog (op/dialog/dialog.py — 하위 클래스는 op/dialog/<이름>_dialog.py)
 ├─ InputDialog      한 줄 입력 · validate        ← ui.modal_input
 ├─ KeyDialog        키 하나 (confirm의 y/n)      ← ui.modal_key · ui.confirm
 ├─ PagerDialog      결과창 스크롤                ← ui.pager
 ├─ BusyDialog       처리 중 창 (키 루프 없음)    ← ui.busy · step · progress
 ├─ TeleopDialog     텔레오퍼                     ← bus.cmd_teleop (A/B 경로 goal 찍기는 on_mark로 같은 창)
 └─ RouteRvizDialog  A/B RViz goal 찍기          ← pages/ab 경로 편집
```

새 창은 `op/dialog/<이름>_dialog.py`에 `Dialog`를 상속해 위 메서드만 채우고 `dialog/__init__.py`에 넣는다.
쓰는 곳에서는 `XxxDialog(ui, ...).run()`의 결과로 다음 동작을 정한다.

#### 상단 상태바도 칸(State)을 상속해서 더한다

```text
State (op/bars/state/state.py)  — text()만 정함 (빈 문자열 = 그 칸 안 보임)
 ├─ OperationState · MapState · EstopState · DeviceState · ConnectionState     (줄 1)
 ├─ PerceptionState · TaskState · DiagState                        (줄 2)
 ├─ TopicRateState (키·라벨·기준 Hz를 받아 joint·odom·imu·카메라·인지 6개)  (줄 3)
 ├─ RobotState                                                               (줄 4)
 └─ NoDataState  (상태를 하나도 못 받았을 때만)
StateBar (bars/state/state_bar.py)  — State를 줄(row)로 받아 ' | '로 이어 그림 · add(state, row)로 칸·줄 추가
 └─ TopStateBar (bars/state/top_state_bar.py) — 위 구성. Window.top_bar, Window.top_lines()가 그린다
```

새 칸: `op/bars/state/<이름>_state.py`에 `State`를 상속해 `text()`를 만들고 `TopStateBar` 구성에 넣거나 `ui.window.top_bar.add(XxxState(ui), row=1)`.
고를 수 있는 칸으로 만들려면 `focusable = True` + `activate()` (예: `DiagState.activate()` → `self.ui.window.navigate('diagnostics')`).

#### 화면 틀 — Window가 Component들을 조합한다

```text
Component (op/component.py)  — focusable · on_key(k) → 처리했으면 True · activate() → STAY/BACK/MAIN/QUIT 또는 값
 ├─ MenuItem (op/menus/item.py)    activate() = window.perform(label, run) — '처리 중' 창 · print 가로채 결과창 · run()
 │   └─ ChoiceItem (bars/menu/choice_item.py)  (key, label) 고르기 — activate()가 key를 돌려줌 (ui.select)
 ├─ State (bars/state/state.py)  focusable 기본 False — DiagState는 True (Enter → 진단 화면)
 ├─ StateBar → TopStateBar      focusable 칸이 있으면 포커스 가능 · ←→ 칸 이동 · activate() = 그 칸의 activate()
 └─ MenuBar (bars/menu/menu_bar.py)  MenuItem들을 세로로 · ↑↓/숫자/글자 포커스
     └─ LeftSideMenuBar (bars/menu/left_side_menu_bar.py)  왼쪽 열 (폭 = 라벨에 맞춤, 최소 24 · 최대 화면 절반)

Window (op/window/window.py) — ui.window
  top_bar(TopStateBar) + menu_bar(LeftSideMenuBar) + 중앙(center_fn) + 힌트 줄을 조합해 draw()
  navigate(name, **params)                     이름으로 화면 열기 — 웹 페이지 이동처럼 파라미터를 넘김 (앱의 screen(name, **params)가 받음)
  run(title, items_fn, center_fn, back, main)  화면 하나의 키 루프 (Menu.show가 부름)
     Tab = 메뉴 막대 ↔ 상태바 포커스 · Enter = 포커스된 부품 activate() · Esc/b/m/q = 이동
  run(…, center_comp=지도, focus_center=True)  중앙에 포커스 가능한 부품을 올림 — Tab으로 메뉴 막대와 오감 (예: MapPage의 MapView)
  select(title, [(key, label)], center)        한 번 고르기 (ChoiceItem으로 run(once=True))
  perform(label, fn, busy)                     실행 + 끝나면 이 화면의 메뉴 막대·중앙·포커스 복원
  신호 connect/emit — OperatorTool이 연결:
     'tick'(seconds)  ROS 폴링 + 재연결 판단 (메뉴 대기 · 창이 떠 있는 동안)
     'navigate'(name, params) screen(name, **params).show() — 상태 칸·다른 화면에서 화면 열기 (파라미터 전달)
```

소스에서 직접 실행하려면 Enter와 같은 길로 `item.activate()` / `state.activate()`를 부른다.

### 2.4 stdin/stdout — 표준 입출력 그대로, 터미널 모드만 바꿈

터미널 기본은 canonical 모드입니다: Enter를 쳐야 한 줄이 넘어오고, 친 글자가 자동으로 찍힙니다(echo). TUI는 이걸 끕니다.

```python
# op/keys.py KeyReader
old = termios.tcgetattr(fd)
tty.setcbreak(fd)                               # 줄 단위 X · echo X → 키 누르는 즉시 1바이트
...
termios.tcsetattr(fd, termios.TCSADRAIN, old)   # 끝나면 반드시 복구 (with 블록 __exit__)

ready, _, _ = select.select([fd], [], [], 0.1)  # 최대 0.1초만 기다림
raw = os.read(fd, 1)                            # 버퍼 있는 sys.stdin.read는 select와 어긋나 키가 씹힘
```

- `select`로 기다리는 시간을 제한해야 키가 없어도 루프가 돕니다 → 그 사이 ROS 콜백(Window `'tick'` 신호)과 2초 갱신.
- `tcflush`(입력 버퍼 비우기)는 **확인창·입력창·결과창을 열 때만** 합니다 (앞 화면에서 누른 `y`가 확인창을 미리 넘기지 않게). 메뉴(`Window._wait_key`)는 `flush=False` — 2초마다 다시 그릴 때마다 비우면 그 사이 누른 키가 사라집니다.

출력 쪽 규칙:

- 메뉴 항목 실행 중 `print()`는 `contextlib.redirect_stdout`으로 가로채 **결과창(pager)** 에 모아 보여 줍니다 (`Ui.capture`, `Window.perform`). 그냥 찍으면 화면이 스크롤돼 레이아웃이 깨집니다.
- 화면 그리기는 시작할 때 잡아 둔 진짜 stdout(`Ui.console` = `Terminal.out`)에 씁니다 → 가로채기에 안 걸림. 텔레오퍼 HUD도 여기에 그립니다.

### 2.5 색

색도 escape code입니다.

```python
# op/terminal.py
RED, GREEN, YELLOW, CYAN = '\033[31m', '\033[32m', '\033[33m', '\033[36m'
BOLD, DIM, REV, RESET = '\033[1m', '\033[2m', '\033[7m', '\033[0m'

def paint(self, text, color):
    return f'{color}{text}{RESET}' if self.color else text
```

- `\033[31m` 뒤 글자는 빨강, `\033[0m`에서 원래대로. 선택된 메뉴 줄은 반전(`7`) + 청록 + 굵게.
- stdout이 TTY가 아니거나 `NO_COLOR`가 있으면 색을 뺍니다 → `./run-cli status | grep`에 깨진 문자가 안 섞임.

### 2.6 키 해석과 메뉴 이동

방향키는 여러 바이트로 옵니다.

| 키 | 바이트 | 키 | 바이트 |
|---|---|---|---|
| ↑ ↓ | `\x1b[A` `\x1b[B` | PgUp PgDn | `\x1b[5~` `\x1b[6~` |
| ← → | `\x1b[D` `\x1b[C` | Home End | `\x1b[H` `\x1b[F` |
| Esc 단독 | `\x1b` | Enter / Backspace | `\r`·`\n` / `\x7f` |
| Tab | `\t` | | |

Esc 단독과 방향키가 둘 다 `\x1b`로 시작하므로, `\x1b`를 받으면 **40ms 더 기다려** 뒤 바이트가 오면 방향키, 안 오면 Esc로 봅니다. 한글 등 UTF-8은 첫 바이트를 보고 몇 바이트를 더 읽을지 정합니다. `KeyReader.read()`는 `'up'`·`'esc'`·`'enter'`·`'가'` 같은 이름으로 돌려줍니다.

메뉴 막대는 **현재 위치 `idx` 하나**만 들고 있습니다.

```python
# op/bars/menu/menu_bar.py MenuBar.on_key()
if k == 'up':    self.idx = (self.idx - 1) % len(self.items)   # 맨 위에서 ↑ → 맨 아래
if k == 'down':  self.idx = (self.idx + 1) % len(self.items)
# Enter는 Window가: menu_bar.focused_item.activate()
# 다시 그릴 때 i == idx인 줄만 '❯' + 반전색
```

"커서를 옮긴다"는 화면 조작이 아니라 **상태(`idx`)를 바꾸고 전체를 다시 그리는 것**입니다. 화면은 언제나 상태를 그린 결과입니다 (React의 상태 → 렌더와 같은 생각).

숫자는 1초 안에 이어 치면 `1`→`11`처럼 모읍니다. 글자 키는 같은 key 항목으로 포커스만 옮깁니다 (`b m q r`은 내비게이션 전용이라 항목 키로 못 씀 — `NAV_KEYS`).

## 3. 화면 흐름 규약

키 규약은 `op/window/window.py`·`op/dialog/dialog.py` 맨 위, 흐름 규약은 `op/menus/menu.py` 맨 위에 있습니다. 새 화면도 이걸 지킵니다.

- `MenuItem.run()`은 `BACK/MAIN/QUIT/STAY`만 돌려줍니다. 화면 이동은 반환값으로만.
  - 하위 화면이 돌려준 `BACK`은 "그 화면이 닫힘"이라 부모는 남습니다 (두 단계 뒤로 가지 않음). `MAIN`·`QUIT`만 위로 전파.
- 결정 전에 알아야 할 경고·현재값은 `print`가 아니라 **`ui.confirm(body=[...])` / `ui.modal_input(body=[...])`** 에 넣습니다. `print`는 가로채져 확인창 *뒤* 결과창에 나오기 때문입니다.
- 값 검사는 `modal_input(validate=...)`로 — 거부 사유 문자열을 돌려주면 창이 안 닫히고 사유를 보여 줍니다.
- 취소(Esc·아니오)는 아무것도 출력하지 않고 `STAY`.
- 수동 명령 전 모드 확보는 `bus.ensure_manual_or_switch()` / `ensure_arm_or_switch()` 하나로 (True 진행 · False 차단(사유 출력됨) · None 사용자 취소). 거부 문구는 최종 판정에서만 찍힙니다.
- 고를 게 없으면 `InfoItem(app, '안내 문구')`.

### 메뉴 하나 추가하기

```python
# op/pages/예시.py
from ..defs import STAY
from .base import Menu, MenuItem


class HelloItem(MenuItem):
    key = '1'

    def label(self):
        return '인사'

    def run(self):
        if not self.ui.confirm('인사할까요?', body=['결과는 결과창에 나옵니다']):
            return STAY          # 취소 — 출력 없이
        print('안녕하세요')       # → 결과창
        return STAY


class HelloMenu(Menu):
    title = '예시'

    def items(self):
        return [HelloItem(self.app)]

    def center(self):           # 오른쪽 내용 (2초마다 다시 불림)
        return ['[예시]', f' 조작: {self.state.operation or "-"}']
```

부모 메뉴의 `items()`에 `GoMenu(self.app, '키', '라벨', HelloMenu(self.app))`를 넣습니다. 메인 메뉴는 `pages/mainmenu.py`의 `ENTRIES`와 `operator_tool.py`의 `screen()`.

## 4. 시험

```bash
cd tools
./run-cli --source fake                   # 가짜 ROS로 메뉴 직접 눌러 보기
./run-cli --source docker status          # 도커 test-faker가 쓰는 debug/ 파일로 실제 스택 조작
python3 cli/tests/scenarios.py            # 전체 시나리오
python3 cli/tests/scenarios.py teleop-hud
```

- `scenarios.py`는 `pty.fork`로 진짜 터미널을 만들어 `python -m cli --source fake menu`를 띄우고, 키 바이트(`\x1b[B`, `\r`…)를 써 넣은 뒤 화면 출력에서 기대 문자열을 찾습니다. `expect`/`absent`는 **마지막 키를 보낸 뒤의 출력만** 봅니다 (앞 화면에 남은 글자로 통과하지 않게).
- 새 시나리오는 `@scenario('이름')` 함수 하나 — 등록은 데코레이터가 합니다.
- 실패하면 화면 출력이 `cli/tests/.scenario-<이름>.bin`에 남습니다. 색 코드를 빼고 보기:
  `python3 -c "import re,sys;print(re.sub(r'\x1b\[[0-9;?]*[A-Za-z]','',open(sys.argv[1],'rb').read().decode('utf-8','replace')))" cli/tests/.scenario-이름.bin`
- 가짜 ROS(`ros/fake/FakeRos.py`)의 파라미터 클라이언트는 항상 무응답이라 파라미터 화면은 거부 경로만 시험됩니다. docker 모드는 `debug/node/<노드>/params`를 읽어 표시하지만, 변경은 아직 faker에 입력 통로가 없습니다. 진짜 동작은 로봇에서 `./run-cli`로 확인합니다.

## 5. 직접 만든 것의 한계

| 한계 | 지금 | 필요해지면 |
|---|---|---|
| 창 크기 변경 | `SIGWINCH` 안 받음 — 다음 그리기(최대 2초 · 키 입력 즉시)에 반영 | `signal.signal(SIGWINCH, …)`에서 다시 그리기 |
| 다시 그리기 | 매번 화면 전체를 씀 (바뀐 칸만 쓰지 않음) — ssh가 느리면 무거울 수 있음 | 줄 단위로 이전 프레임과 비교해 바뀐 줄만 쓰기, 또는 `curses` |
| Esc 판정 | 40ms 기다림 — 아주 느린 연결에서 방향키가 Esc로 잘못 읽힐 수 있음 | 대기 시간 늘리기 |
| 터미널 종류 | xterm 계열 escape code 가정 (macOS Terminal·iTerm·리눅스 기본 터미널·ssh OK) | `curses`(terminfo) |
| 입력창 | 커서 이동·붙여넣기 편집 없음 (Backspace·Ctrl+U만) | `prompt_toolkit` |

## 5. 지도 화면 (MapPage) — 파라미터로 쓰임이 바뀌는 화면

`pages/map/` 하나가 **보기 · A/B 경로 goal 찍기 · 로봇 위치 지정**을 다 한다. 웹 페이지처럼 이름과 파라미터로 연다:

```python
self.ui.window.navigate('map', mode='view')                                   # 지금 /map 보기 (로봇 ● 표시)
self.ui.window.navigate('map', mode='view', source=map_yaml)                  # 저장된 지도 파일 보기
self.ui.window.navigate('map', mode='goals', start=…, goals=…, on_save=cb)    # A/B 경로 — 저장하면 cb(goals)
self.ui.window.navigate('map', mode='pose', on_pose=fn)                       # 위치 지정 — Enter → fn(x, y, yaw_deg)
```

- 중앙 = `MapView`(포커스 가능 Component), 왼쪽 = 같은 동작 메뉴. Tab/Esc로 오간다. 화면 한가운데 십자선(+)이 찍을 자리, 로봇은 빨간 점(●), 찍는 모드는 십자선에서 방향선이 나간다 (`[` `]` 15° · `{` `}` 1°).
- 로봇 따라가기(기본): 중심이 로봇을 따라간다. 지도가 한 화면에 다 보이면 지도는 고정되고 로봇 점이 움직인다. 방향키로 지도를 밀면 풀리고 `r`로 다시 켠다.
- 새 쓰임은 mode를 늘리지 말고, 먼저 파라미터(`on_pose` 같은 콜백)로 되는지 본다.

## 6. 팔 조작 화면 (ArmControlPage) — 숫자 없이, 확인 뒤에만 움직임

`pages/arm/`(`window.navigate('arm')`, 팔 메뉴 `0)`). 중앙 = `ArmView`, 왼쪽 = 동작 메뉴. 관절마다 한계 범위 막대 위에 현재(●)와 목표(◆).

- `↑↓` 관절 · `←→` 목표 ∓2° · `,` `.` ∓1° · `z` 목표를 현재값으로. **목표만 바뀐다 — 팔은 안 움직인다.**
- `Enter`(또는 왼쪽 '적용') → 바꾼 관절의 이동량 `현재° → 목표° (Δ)`를 보여 주는 확인창(**기본 아니오**, 30° 넘게 움직이면 경고) → 예일 때만 `Bus.cmd_arm(set)`으로 전송.
- 전송 경로는 기존 `cmd_arm` 그대로라 MANUAL 전환 확인 · MAPPING/E-STOP 거절 · manual_control 쪽 속도 제한이 그대로 적용된다.
- 목표는 `consts.ARM_LIMITS`(URDF `<limit lower upper>`와 같은 값) 밖으로 못 간다. `tests/test_arm_view.py`가 URDF와 어긋나면 알려 준다.
- 새 관절·그리퍼를 더하려면 `ArmView(joints, limits, current_fn)`에 이름과 한계를 넘긴다.
