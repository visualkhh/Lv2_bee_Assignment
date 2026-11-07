# 원격 데스크톱 — VNC (x11vnc, 기본 방법)

Jetson의 **로그인된 실제 데스크톱(`:0`)** 을 그대로 공유한다. GPU로 그린 화면이라 RViz도 쓸 만하다.
모니터가 없어도 된다 — `run-vnc-server --headless`를 한 번 하면 X에 가상 모니터가 꽂힌다([0장](#0-모니터-없이-headless-한-번)).
VNC가 안 될 때만 [RDP.md](RDP.md).

[← README.md](README.md) (SSH 키 등록 · 방법 고르기)

## 한눈에 보기

| 구분 | Jetson (화면을 공유하는 쪽) | PC (화면을 보는 쪽) |
| --- | --- | --- |
| 역할 | 실제 Ubuntu 데스크톱을 가진 서버 | 원격 화면을 보는 클라이언트 |
| 필요한 것 | `x11vnc`, 로그인된 데스크톱 | SSH 클라이언트, Remmina + VNC 플러그인 |
| 실행하는 것 | `x11vnc` | SSH 터널, Remmina |
| 주소 | `127.0.0.1:5900` | `localhost:5900` |

핵심은 간단하다.

```text
Jetson: Ubuntu 데스크톱 로그인 → x11vnc가 :0 화면을 공유
PC    : SSH 터널로 Jetson의 5900에 연결 → Remmina VNC로 화면 표시
```

### 설정 순서

1. **Jetson**: 자동 로그인 또는 모니터에서 `bee`로 그래픽 로그인 (모니터 없으면 `--headless` 한 번)
2. **Jetson**: `x11vnc` 설치 및 VNC 비밀번호 생성
3. **Jetson**: `x11vnc` 실행 (`127.0.0.1:5900`)
4. **PC**: SSH 터널 열기
5. **PC**: Remmina VNC → `localhost:5900`

## 0. 모니터 없이: --headless (한 번)

모니터 없이 부팅하면 Jetson X(nvidia, `AllowEmptyInitialConfiguration`)는 640x480 빈 화면으로 뜨지만
**GNOME이 모니터 0개라 아무것도 그리지 않는다 → VNC가 검은 화면**이다(실측 10/26). `xrandr --setmonitor`로
가상 모니터를 등록해도 GNOME은 무시한다(실측). 그래서 nvidia 드라이버 옵션으로 DP(`DFP-0`)에 모니터가
꽂힌 것처럼 만든다 = **소프트웨어 더미 플러그**. 하드웨어 더미 플러그는 필요 없다.

Jetson에서 한 번 (sudo 비밀번호 필요 — GDM 재시작, SSH에서 띄운 ROS는 그대로):

```bash
~/run-vnc-server --headless
# 15초쯤 뒤 (GDM 재시작으로 x11vnc가 꺼짐)
~/run-vnc-server
```

`--headless`가 하는 일:

1. 가짜 모니터 정보 `/etc/X11/edid-bee-virtual.bin` (EDID 128바이트 — 이름 `BEE-VIRTUAL`, 1920x1080·1280x720)
2. `/etc/X11/xorg.conf`의 nvidia Device 섹션에 두 줄 (원본 `xorg.conf.run-vnc.bak`):
   ```text
   Option "ConnectedMonitor" "DFP-0"
   Option "CustomEDID" "DFP-0:/etc/X11/edid-bee-virtual.bin"
   ```
3. GDM 재시작 → 자동 로그인 데스크톱이 가상 모니터 위에 뜬다

확인 (`run-vnc-server`가 해상도를 1280x720으로 바꾼다):

```bash
grep -E "BEE-VIRTUAL|DFP-0" /var/log/Xorg.0.log | tail -3     # BEE-VIRTUAL (DFP-0): connected
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority xrandr --listmonitors   # Monitors: 1 · DP-0
```

- 재부팅해도 유지된다. 재부팅·GDM 재시작 뒤에는 x11vnc만 다시 띄운다(`~/run-vnc-server`).
- 진짜 모니터를 꽂아 쓸 때는 `~/run-vnc-server --headless-undo` (가상 EDID가 진짜 모니터 정보를 덮으므로).
- 데스크톱이 안 뜨면 SSH로 `~/run-vnc-server --headless-undo` → 원래대로.
- 자동 로그인이 켜져 있어야 한다([2장](#2-키보드-없이-로그인-gdm-자동-로그인)). RDP(`run-rdp-server`)를 설정했었다면 먼저 `~/run-rdp-server --undo`.

## 빠른 실행 (스크립트)

### Jetson: run-vnc-server

Jetson에 스크립트를 복사해서 실행한다.

```bash
scp documents/jetson-orin-nano/remote/run-vnc-server bee@bee.local:~/run-vnc-server
ssh bee@bee.local 'chmod +x ~/run-vnc-server && ~/run-vnc-server'
```

이미 Jetson에 저장소가 있으면 그 경로에서 실행해도 된다.

```bash
cd ~/Lv2_bee_Assignment/documents/jetson-orin-nano/remote
./run-vnc-server
```

기본 동작:

```text
주소      : 127.0.0.1:5900
display   : :0
비밀번호  : ~/.vnc/passwd
백그라운드 실행, 로그 ~/.vnc/x11vnc.log
```

자주 쓰는 명령:

```bash
./run-vnc-server --start
./run-vnc-server --status
./run-vnc-server --stop
./run-vnc-server --restart
./run-vnc-server --restart --xrandr 1280x720   # 실제 출력 해상도 변경 (모니터에도 적용)
./run-vnc-server --scale 1280x720
./run-vnc-server --foreground
```

`run-vnc-server`는 항상 `-localhost`로 실행한다. PC에서 직접 `10.2.12.110:5900`으로
붙는 방식이 아니므로 아래 `run-vnc-client`의 SSH 터널이 필요하다.

### PC: run-vnc-client

PC에서 실행한다.

```bash
cd documents/jetson-orin-nano/remote
./run-vnc-client
```

기본 동작:

```text
SSH 대상   : bee@bee.local
PC 포트    : localhost:5900
Jetson VNC : 127.0.0.1:5900
```

SSH 키가 이미 등록되어 있어야 한다. 접속 대상이 다르면:

```bash
./run-vnc-client --host bee@10.2.12.110
```

PC의 5900 포트가 이미 사용 중이면:

```bash
./run-vnc-client --local-port 5901
```

터널만 열고 VNC 클라이언트는 직접 실행하려면:

```bash
./run-vnc-client --no-vnc
```

상태/종료:

```bash
./run-vnc-client --start
./run-vnc-client --status
./run-vnc-client --stop
./run-vnc-client --restart
```

VNC 비밀번호는 Jetson 로그인 비밀번호가 아니라 `x11vnc -storepasswd`에서 만든
비밀번호다.

## 꼭 구분할 것

- `x11vnc`는 **Jetson에서만** 실행한다. PC에서 실행하는 프로그램이 아니다.
- SSH 터널과 Remmina는 **PC에서만** 실행한다.
- `-localhost`를 쓰면 Jetson의 5900은 외부에 열리지 않는다. 그래서 PC에서 SSH 터널이 필요하다.
- VNC 비밀번호는 `x11vnc -storepasswd`에서 만든 비밀번호다. Jetson 로그인 비밀번호와 다르다.

## 1. Jetson: x11vnc로 화면 공유

Jetson에 모니터가 연결되어 있고, 실제로 로그인한 GNOME 화면을 그대로 공유할 때
사용한다. **Jetson 모니터에서 `bee`로 로그인한 상태여야 한다.** GDM 로그인 화면은
일반 사용자 권한으로 공유하지 않는다.

Jetson에 `x11vnc`를 설치한다.

```bash
sudo apt update
sudo apt install -y x11vnc
```

Jetson 모니터의 데스크톱 터미널에서 데스크톱이 X11인지 확인한다.

```bash
echo "DISPLAY=$DISPLAY"
echo "XDG_SESSION_TYPE=$XDG_SESSION_TYPE"
```

`DISPLAY=:0`, `XDG_SESSION_TYPE=x11`이면 아래 명령을 그대로 쓸 수 있다.
Wayland 세션이면 x11vnc를 쓸 수 없다 — 로그인 화면 톱니바퀴에서 "Ubuntu on Xorg"를 고른다.

VNC 비밀번호 파일을 만든다. `~/.vnc/passwd`가 생성된다.

```bash
mkdir -p ~/.vnc
x11vnc -storepasswd
```

Jetson에서 x11vnc를 실행한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

- `-display :0` : 실제 GNOME 데스크톱
- `-auth guess` : Xauthority 자동 탐색
- `-forever` : 클라이언트가 끊겨도 서버 유지
- `-shared` : 여러 클라이언트 동시 접속 허용
- `-localhost` : 로컬에서만 listen. 원격은 SSH 터널로 접속
- `-rfbauth` : VNC 비밀번호 파일

`-auth guess`가 실패하면 실제 Xauthority 경로를 지정한다.

```bash
# 예: 사용자 세션의 Xauthority
x11vnc -display :0 -auth "$HOME/.Xauthority" -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

Jetson에서 5900 포트가 열렸는지 확인한다.

```bash
ss -ltnp | grep 5900
```

### PC: SSH 터널 + VNC 접속

PC에서 SSH 터널을 연다. 이 터미널은 접속 중 계속 열어 둔다.

```bash
ssh -N -L 5900:127.0.0.1:5900 bee@bee.local
```

이 명령의 의미:

```text
PC의 localhost:5900
  → SSH 터널
  → Jetson의 127.0.0.1:5900 (x11vnc)
```

PC에 VNC 클라이언트를 설치한다.

```bash
sudo apt install -y remmina remmina-plugin-vnc
# 또는
sudo apt install -y tigervnc-viewer
```

#### Remmina 명령어로 바로 접속

SSH 터널이 열려 있는 상태에서:

```bash
remmina -c vnc://localhost:5900
```

`vnc://`를 반드시 붙인다. `rdp://`가 아니고, 웹 브라우저 주소창에 넣는 것도 아니다.

터널을 백그라운드로 열고 Remmina까지 한 번에 실행하려면:

```bash
ssh -f -N -L 5900:127.0.0.1:5900 bee@bee.local
remmina -c vnc://localhost:5900
```

이미 5900 터널이 떠 있으면 두 번째 줄만 실행한다.
포트가 이미 사용 중이면 터널 포트를 5901로 바꾸고:

```bash
ssh -N -L 5901:127.0.0.1:5900 bee@bee.local
remmina -c vnc://localhost:5901
```

VNC 클라이언트에서 직접 주소를 입력할 때는:

```text
localhost:5900
```

`x11vnc -storepasswd`에서 설정한 VNC 비밀번호를 입력하면 Jetson의 전체 데스크톱이
나온다.

종료는 x11vnc를 실행한 Jetson 터미널에서 `Ctrl-C`, 또는:

```bash
pkill x11vnc
```

### 성능: 해상도 낮추기

> `run-vnc-server` 기본값은 저부하 프리셋이다: 실제 출력 해상도 `1280x720` 자동
> 적용(물리 모니터에도 같이 적용됨), `-scale 960x540`, `-speeds dsl`,
> `-wireframe`, `-sb 15`, **GNOME 애니메이션 끄기**(`--animations on`으로 다시 켬). 자동 변경을 끄려면
> `./run-vnc-server --restart --xrandr keep`. 색상 깊이(8/16-bit)는 VNC 프로토콜상 클라이언트가
> 정하므로 서버에서 강제할 수 없다 — `run-vnc-client`가 **기본으로 16bpp · 품질 medium** 접속 설정 파일로
> Remmina를 띄운다 (`--bpp 8`, `--quality poor`로 더 낮춤 · `--plain`이면 Remmina 기본값).

1920x1080 화면을 32-bit로 보내면 VNC가 느려지기 쉽다. Jetson에서 실제 해상도를
1280x720으로 낮추면 가장 효과가 크다.

Jetson SSH에서:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# 현재 연결된 출력 이름 확인 (예: DP-0)
xrandr --current

# 1280x720으로 변경
xrandr --output DP-0 --mode 1280x720
```

출력 이름은 `xrandr --current`의 `connected primary` 줄에서 확인한다.
예시에서는 `DP-0`을 사용했다.

해상도를 바꾼 뒤 x11vnc를 재시작한다.

```bash
pkill x11vnc

x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost \
  -wait 20 -defer 40
```

다시 1920x1080으로 되돌리려면:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
xrandr --output DP-0 --mode 1920x1080
```

해상도 자체를 바꾸지 않고 VNC 서버에서만 축소하려면 `-scale`을 사용한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost \
  -scale 1280x720 -wait 20 -defer 40
```

Remmina 쪽에서도 연결 편집 → **고급 설정**에서:

```text
색상 깊이(Color depth): 8-bit 또는 16-bit
품질(Quality): Low/Poor
Tight encoding: 켬
```

로 맞추면 전송량이 크게 줄어든다.
`colordepth=32`는 가능하면 피한다.

> `-localhost`를 빼고 `0.0.0.0`에 열면 Wi-Fi/LAN의 다른 PC가 바로 접속할 수 있다.
> 보안상 권장하지 않으며, 외부 공유기 포트 포워딩은 절대 하지 않는다.
> 원격지에서 써야 하면 SSH 터널이나 Tailscale/WireGuard를 사용한다.

### x11vnc 자동 시작 (선택)

로그인 후 자동으로 x11vnc를 띄우려면 systemd 사용자 서비스를 만든다.
먼저 `x11vnc -storepasswd`로 `~/.vnc/passwd`를 만들어 둔다.

```bash
mkdir -p ~/.config/systemd/user

cat > ~/.config/systemd/user/x11vnc.service <<'EOF'
[Unit]
Description=x11vnc desktop sharing
After=graphical-session.target

[Service]
ExecStart=/usr/bin/x11vnc -display :0 -auth guess -forever -shared -rfbauth %h/.vnc/passwd -localhost
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable --now x11vnc.service
systemctl --user status x11vnc.service
```

자동 시작을 끄려면:

```bash
systemctl --user disable --now x11vnc.service
```

## 2. 키보드 없이 로그인 (GDM 자동 로그인)

x11vnc는 `bee`가 GNOME 데스크톱에 **이미 로그인한 뒤에만** 사용할 수 있다.
GDM 로그인 화면은 `gdm` 사용자 권한으로 실행되므로, SSH에서 `bee`로 x11vnc를
띄우면 보통 다음처럼 실패한다.

```text
Authorization required, but no authorization protocol specified
xset: unable to open display ":0"
```

> 모니터가 없으면 자동 로그인만으로는 GNOME이 화면을 안 그린다(검은 화면) — [0장](#0-모니터-없이-headless-한-번) `--headless`.
> RDP(`run-rdp-server`)를 설정하면 자동 로그인이 꺼진다 — VNC로 돌아오려면 `~/run-rdp-server --undo`.

Jetson이 부팅될 때 `bee`로 자동 로그인하게 만든다. 보안상 **임시방편**으로만
사용하고, 원격 접속이 확인되면 자동 로그인은 끄는 것을 권장한다.

```bash
ssh -t bee@bee.local 'sudo nano /etc/gdm3/custom.conf'
```

`[daemon]` 아래에 추가한다.

```ini
[daemon]
AutomaticLoginEnable=True
AutomaticLogin=bee
```

저장한 뒤 GDM을 재시작한다.

```bash
ssh -t bee@bee.local 'sudo systemctl restart gdm'
```

잠시 기다린 뒤 `bee`의 그래픽 세션이 생겼는지 확인한다.

```bash
ssh bee@bee.local 'loginctl list-sessions; ps -eo pid,user,args | grep "[X]org"'
```

`seat0` 세션이 `active`이고 Xorg 프로세스가 보이면 x11vnc를 실행할 수 있다.
`DISPLAY=:0`은 Jetson 모니터의 데스크톱 터미널에서 확인한다.

```bash
x11vnc -display :0 -auth guess -forever -shared \
  -rfbauth ~/.vnc/passwd -localhost
```

PC에서:

```bash
ssh -N -L 5900:localhost:5900 bee@bee.local
```

Remmina VNC로 `localhost:5900`에 접속한다.

자동 로그인을 끄려면 `custom.conf`에서 두 줄을 제거하거나 주석 처리한 뒤:

```bash
ssh -t bee@bee.local 'sudo systemctl restart gdm'
```

> 자동 로그인은 Jetson에 물리적으로 접근할 수 있는 사람에게 데스크톱을 바로
> 넘긴다. 공용 로봇이나 외부 네트워크에 연결된 장비에서는 오래 켜 두지 않는다.

## 3. 체크리스트

```text
[ ] (모니터 없음) Jetson ~/run-vnc-server --headless 한 번 → Xorg.0.log에 BEE-VIRTUAL connected
[ ] Jetson 자동 로그인(또는 모니터에서 bee 그래픽 로그인)
[ ] Jetson 데스크톱 터미널에서 echo $DISPLAY → :0, XDG_SESSION_TYPE=x11
[ ] x11vnc -storepasswd로 ~/.vnc/passwd 생성
[ ] Jetson ./run-vnc-server (또는 x11vnc ... -localhost)
[ ] ss -ltnp | grep 5900 확인
[ ] PC ./run-vnc-client (또는 ssh -N -L 5900:127.0.0.1:5900 bee@bee.local → remmina -c vnc://localhost:5900)
[ ] (그래도 느리면) ./run-vnc-client --bpp 8 --quality poor
```

문제가 생기면 먼저 확인할 것:

```bash
# x11vnc 로그/포트
./run-vnc-server --status
ss -ltnp | grep 5900
ps -ef | grep '[x]11vnc'
tail -30 ~/.vnc/x11vnc.log

# 데스크톱 세션
echo "$DISPLAY $XDG_SESSION_TYPE"
loginctl list-sessions
```
