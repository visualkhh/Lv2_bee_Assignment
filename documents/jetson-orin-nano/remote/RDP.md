# 원격 데스크톱 — RDP (xrdp, VNC가 안 될 때)

> **기본은 [VNC.md](VNC.md)** — 모니터가 없어도 `run-vnc-server --headless`로 GPU로 그린 실제 데스크톱을
> 쓸 수 있다(실측 10/26). RDP는 그게 안 될 때의 대안이다. RDP를 설정하면 자동 로그인이 꺼져 VNC를 못 쓰고,
> VNC로 돌아가려면 `~/run-rdp-server --undo`.

Jetson에 **모니터가 없고 VNC가 안 될 때** 쓴다. Jetson은 `run-rdp-server`로 한 번 설정하고, PC는 `run-rdp-client`로
접속한다. xrdp는 RDP 접속마다 **자기 가상 X 화면**을 만들어서 모니터·Wayland·nvidia 드라이버와 상관없이 된다.
대신 GPU 없이 CPU로 그려서 느리다([5장](#5-느릴-때)). [VNC.md](VNC.md)가 빠르다.

[← README.md](README.md) (SSH 키 등록 · 방법 고르기)

```text
Jetson: run-rdp-server (한 번) → xrdp가 3389에서 대기
PC    : run-rdp-client → SSH 터널 localhost:3389 → Remmina RDP → xrdp 로그인(세션 Xorg) → 가상 GNOME 데스크톱
```

> 우분투 기본 RDP(GNOME 원격 로그인, `grdctl --system`)는 Wayland 전용이다. Jetson은
> `/etc/gdm3/custom.conf`에 `WaylandEnable=false`라서 접속하면 **NVIDIA 로고에서 멈추고 키 입력에
> 반응이 없다**(실측 10/26). 그래서 그쪽은 끄고 3389 포트를 xrdp에 준다.

## 1. Jetson: run-rdp-server (한 번)

Jetson에서 (sudo 비밀번호 필요):

```bash
scp documents/jetson-orin-nano/remote/run-rdp-server bee@bee.local:~/run-rdp-server
ssh -t bee@bee.local '~/run-rdp-server'
```

스크립트가 하는 일:

1. `xrdp`, `xorgxrdp` 설치, `xrdp`를 `ssl-cert` 그룹에 추가 (TLS 키 읽기)
2. GNOME 원격 데스크톱(시스템 RDP) 끄기 — 3389 포트 충돌 방지
3. `~/.xsessionrc`에 Ubuntu GNOME 세션 환경 추가 (로컬 로그인과 같은 모양).
   Jetson 기본 `~/.xsessionrc`(NVIDIA, bash 문법)는 `~/.xsessionrc.bash`로 옮겨 bash일 때만 불러옴 (아래 주의)
4. polkit 규칙 — 접속 때 "color managed device" 인증 팝업 방지
5. **GDM 자동 로그인 끄기** — 같은 사용자가 이미 GNOME에 로그인해 있으면 xrdp가 검은 화면
   (원본은 `/etc/gdm3/custom.conf.setup-rdp.bak`)
6. xrdp 시작 → GDM 재시작 (자동 로그인 세션 종료 — SSH에서 띄운 ROS는 그대로)

## 2. PC: run-rdp-client

PC에서 접속한다 (SSH 터널 + Remmina RDP).

```bash
cd documents/jetson-orin-nano/remote
./run-rdp-client               # 터널 localhost:3389 → Jetson 3389, Remmina 실행
./run-rdp-client --status      # 터널 상태
./run-rdp-client --stop        # 터널 종료
./run-rdp-client --no-client   # 터널만 (RDP 클라이언트는 직접)
```

Remmina를 직접 쓸 때:

```text
프로토콜: RDP - 원격 데스크톱 프로토콜
서버    : bee.local:3389 (또는 10.2.12.110:3389)
사용자  : bee / Jetson 계정 비밀번호
도메인  : 비움
```

xrdp 로그인 창이 뜨면 세션 `Xorg`로 로그인한다. LAN 밖에서는 SSH 터널을 쓴다
(`ssh -N -L 3389:localhost:3389 bee@bee.local` → `localhost:3389`). 3389를 외부 인터넷에 열지 않는다.

## 3. 상태 확인 · 되돌리기

```bash
ssh bee@bee.local '~/run-rdp-server --status'   # xrdp · 3389 · 자동 로그인 · 세션 확인
ssh -t bee@bee.local '~/run-rdp-server --undo'  # 되돌리기 (xrdp 끔, 자동 로그인 복구)
```

## 4. 주의 · 안 될 때

- 자동 로그인을 끄므로 그동안 VNC(`run-vnc-server`)는 못 쓴다 — VNC로 돌아가려면 `~/run-rdp-server --undo`
  (xrdp · xrdp-sesman · 남은 xrdp 세션까지 정리, 자동 로그인 복구).
- xrdp 화면은 GPU 없이 그린다 — 느리면 [5장](#5-느릴-때).
- 검은 화면이면 bee의 다른 그래픽 세션이 남았는지 `loginctl list-sessions`로 확인하고 로그아웃 뒤 다시 접속한다.
- 로그인하자마자 끊기면 `sudo tail -30 /var/log/xrdp-sesman.log`(`Window manager … exited quickly`)와
  `~/.xsession-errors`를 본다. Jetson 기본 `~/.xsessionrc`(NVIDIA)는 bash 문법인데 xrdp는 `/bin/sh`로 읽어서
  `Syntax error: "(" unexpected`로 끝난다 — `run-rdp-server`가 원래 내용을 `~/.xsessionrc.bash`로 옮겨
  bash일 때(GDM 로그인)만 불러오게 고친다 (실측 10/26).

## 5. 느릴 때

가상 화면은 **GPU 없이 CPU(llvmpipe)로 그린다**(실측 10/26: `DISPLAY=:10 glxinfo -B` → `llvmpipe`,
`Accelerated: no`). GNOME 셸의 효과·애니메이션까지 CPU가 그리고, 같은 CPU를 ROS 노드와 나눠 쓴다.
효과가 큰 순서:

1. **GNOME 애니메이션 끄기** — `run-rdp-server`가 **기본으로 끈다** (설치 때, 또는 sudo 없이
   `~/run-rdp-server --tune`). 사용자 설정이라 모니터 로그인에도 같이 적용된다.
   다시 켜기: `~/run-rdp-server --tune --animations on`
2. **Remmina 설정 낮추기** — `run-rdp-client`가 **기본으로** 접속 설정 파일을 만들어 접속한다:
   ```text
   해상도      : 1280x720        (--size WxH)
   색 깊이     : 16 bpp          (--bpp 8|15|16|24|32)
   품질        : medium          (--quality poor|medium|good|best)
   네트워크    : LAN
   ```
   더 가볍게 `./run-rdp-client --quality poor`, 선명하게 `./run-rdp-client --size 1920x1080 --bpp 24 --quality good`,
   Remmina 기본값 그대로 `./run-rdp-client --plain`.
3. **가벼운 데스크톱** — xrdp 세션만 GNOME 대신 XFCE (`sudo apt install -y xfce4` 후
   `~/.xsessionrc`의 GNOME 환경 줄 대신 `~/.xsession`에 `startxfce4`). GNOME 셸보다 훨씬 가볍다 (아직 Jetson에서 미검증).
4. RViz처럼 3D가 많은 화면은 RDP로 보지 말고 [VNC.md](VNC.md) (`--headless` — GPU로 그림).

## 6. 체크리스트

```text
[ ] Jetson ~/run-rdp-server 실행 (sudo)
[ ] ~/run-rdp-server --status 에서 xrdp active · 3389 LISTEN 확인
[ ] PC ./run-rdp-client → 사용자 bee · 비밀번호 · 도메인 비움
[ ] xrdp 창에서 세션 Xorg 로그인
[ ] (그래도 느리면) ./run-rdp-client --quality poor · RViz는 VNC (--headless)
```

문제가 생기면 먼저 확인할 것:

```bash
ssh bee@bee.local '~/run-rdp-server --status'
ss -ltnp | grep 3389                       # Jetson
sudo tail -30 /var/log/xrdp-sesman.log     # Jetson — 세션이 바로 끝나는지
tail -20 ~/.xsession-errors                # Jetson — 데스크톱 시작 오류
./run-rdp-client --status                  # PC — 터널
```
