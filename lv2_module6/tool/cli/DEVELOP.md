# DEVELOP — operator_tool(CLI) 개발자용

CLI를 고치거나 화면·메뉴를 추가하는 개발자용 문서입니다. 쓰는 법(명령·키)은 [../README.md](../README.md)를 봅니다.

- Python 3 · rclpy만 씀 (curses·textual 같은 TUI 라이브러리 없음)
- 화면은 **stdout에 글자 + ANSI escape code**를 쓰고, 키는 **stdin에서 바이트를 직접** 읽어서 만든다
- ROS 없이 시험: `../cli-test/` (가짜 ROS층 + pty 키 입력 시나리오)

## 1. 구조

```text
cli/
├── operator_tool.py   # OperatorTool(Node): 구독 배선 + argparse + main. 화면 번호 → Page (screen())
└── op/
    ├── terminal.py    # Terminal — 크기 · 색 · 표시 너비(fit). ROS·메뉴를 모름
    ├── dialogs.py     # KeyReader(키 읽기) · Dialog(확인/입력/결과창) · run_select(메뉴 루프). 키 규약 정리
    ├── ui.py          # Ui — 4분할 프레임 그리기 · 상단 상태바 · confirm/modal_input/pager/select 진입점
    ├── state.py       # StateStore — 구독한 ROS 상태 보관 (화면은 여기만 읽음)
    ├── bus.py         # Bus — 발행·대기·모드 게이트 · cmd_* (한 번 실행 명령과 메뉴가 같이 씀)
    ├── defs.py        # BACK/MAIN/QUIT/STAY/TIMEOUT · MENU_REFRESH_S
    ├── consts.py      # 관절 이름 · 파라미터 변환
    └── pages/         # Page → Menu → 각 화면 (MenuItem 객체)
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
# op/dialogs.py Dialog.draw()
console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')    # top행 left열로 이동 후 쓰기
for i, s in enumerate(content):
    console.write(f'\033[{top + 1 + i};{left}H│' + fit(s, w - 2) + '│')
```

- 뒤의 메뉴 위에 박스를 덮어 그려서 팝업처럼 보입니다. 닫히면 메뉴를 다시 그려 덮습니다.
- 입력창에 진짜 위젯은 없습니다. 키 하나마다 `buf`에 붙이고 박스를 다시 그려 `값> 0.3▏`처럼 보여 줄 뿐입니다 (`▏`도 글자).
- 창 크기가 바뀌어 박스 위치가 달라지면 예전 자리를 공백으로 지웁니다 (`Ui._erase_rect`).

### 2.4 stdin/stdout — 표준 입출력 그대로, 터미널 모드만 바꿈

터미널 기본은 canonical 모드입니다: Enter를 쳐야 한 줄이 넘어오고, 친 글자가 자동으로 찍힙니다(echo). TUI는 이걸 끕니다.

```python
# op/dialogs.py KeyReader
old = termios.tcgetattr(fd)
tty.setcbreak(fd)                               # 줄 단위 X · echo X → 키 누르는 즉시 1바이트
...
termios.tcsetattr(fd, termios.TCSADRAIN, old)   # 끝나면 반드시 복구 (with 블록 __exit__)

ready, _, _ = select.select([fd], [], [], 0.1)  # 최대 0.1초만 기다림
raw = os.read(fd, 1)                            # 버퍼 있는 sys.stdin.read는 select와 어긋나 키가 씹힘
```

- `select`로 기다리는 시간을 제한해야 키가 없어도 루프가 돕니다 → 그 사이 ROS 콜백(`Ui._poll`)과 2초 갱신.
- `tcflush`(입력 버퍼 비우기)는 **확인창·입력창·결과창을 열 때만** 합니다 (앞 화면에서 누른 `y`가 확인창을 미리 넘기지 않게). 메뉴(`run_select`)는 `flush=False` — 2초마다 다시 그릴 때마다 비우면 그 사이 누른 키가 사라집니다.

출력 쪽 규칙:

- 메뉴 항목 실행 중 `print()`는 `contextlib.redirect_stdout`으로 가로채 **결과창(pager)** 에 모아 보여 줍니다 (`Ui.capture`, `Menu.show`). 그냥 찍으면 화면이 스크롤돼 레이아웃이 깨집니다.
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

Esc 단독과 방향키가 둘 다 `\x1b`로 시작하므로, `\x1b`를 받으면 **40ms 더 기다려** 뒤 바이트가 오면 방향키, 안 오면 Esc로 봅니다. 한글 등 UTF-8은 첫 바이트를 보고 몇 바이트를 더 읽을지 정합니다. `KeyReader.read()`는 `'up'`·`'esc'`·`'enter'`·`'가'` 같은 이름으로 돌려줍니다.

메뉴는 **현재 위치 `idx` 하나**만 들고 있습니다.

```python
# op/dialogs.py run_select()
if k == 'up':    idx = (idx - 1) % len(items)   # 맨 위에서 ↑ → 맨 아래
if k == 'down':  idx = (idx + 1) % len(items)
if k == 'enter': return items[idx][0]           # 고른 항목의 key
# 다시 그릴 때 i == idx인 줄만 '❯' + 반전색
```

"커서를 옮긴다"는 화면 조작이 아니라 **상태(`idx`)를 바꾸고 전체를 다시 그리는 것**입니다. 화면은 언제나 상태를 그린 결과입니다 (React의 상태 → 렌더와 같은 생각).

숫자는 1초 안에 이어 치면 `1`→`11`처럼 모읍니다. 글자 키는 같은 key 항목으로 포커스만 옮깁니다 (`b m q r`은 내비게이션 전용이라 항목 키로 못 씀 — `NAV_KEYS`).

## 3. 화면 흐름 규약

키 규약은 `op/dialogs.py` 맨 위, 흐름 규약은 `op/pages/base.py` 맨 위에 있습니다. 새 화면도 이걸 지킵니다.

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
cd tool
./run-cli-test                       # 가짜 ROS로 메뉴 직접 눌러 보기
python3 cli-test/scenarios.py        # 전체 시나리오
python3 cli-test/scenarios.py teleop-hud
```

- `scenarios.py`는 `pty.fork`로 진짜 터미널을 만들어 `cli_test.py menu`를 띄우고, 키 바이트(`\x1b[B`, `\r`…)를 써 넣은 뒤 화면 출력에서 기대 문자열을 찾습니다. `expect`/`absent`는 **마지막 키를 보낸 뒤의 출력만** 봅니다 (앞 화면에 남은 글자로 통과하지 않게).
- 새 시나리오는 `@scenario('이름')` 함수 하나 — 등록은 데코레이터가 합니다.
- 실패하면 화면 출력이 `cli-test/.scenario-<이름>.bin`에 남습니다. 색 코드를 빼고 보기:
  `python3 -c "import re,sys;print(re.sub(r'\x1b\[[0-9;?]*[A-Za-z]','',open(sys.argv[1],'rb').read().decode('utf-8','replace')))" cli-test/.scenario-이름.bin`
- 가짜 ROS(`cli-test/fake_ros.py`)의 파라미터 클라이언트는 항상 무응답이라 파라미터 화면은 거부 경로만 시험됩니다. 진짜 동작은 로봇에서 `./run-cli`로 확인합니다.

## 5. 직접 만든 것의 한계

| 한계 | 지금 | 필요해지면 |
|---|---|---|
| 창 크기 변경 | `SIGWINCH` 안 받음 — 다음 그리기(최대 2초 · 키 입력 즉시)에 반영 | `signal.signal(SIGWINCH, …)`에서 다시 그리기 |
| 다시 그리기 | 매번 화면 전체를 씀 (바뀐 칸만 쓰지 않음) — ssh가 느리면 무거울 수 있음 | 줄 단위로 이전 프레임과 비교해 바뀐 줄만 쓰기, 또는 `curses` |
| Esc 판정 | 40ms 기다림 — 아주 느린 연결에서 방향키가 Esc로 잘못 읽힐 수 있음 | 대기 시간 늘리기 |
| 터미널 종류 | xterm 계열 escape code 가정 (macOS Terminal·iTerm·리눅스 기본 터미널·ssh OK) | `curses`(terminfo) |
| 입력창 | 커서 이동·붙여넣기 편집 없음 (Backspace·Ctrl+U만) | `prompt_toolkit` |
