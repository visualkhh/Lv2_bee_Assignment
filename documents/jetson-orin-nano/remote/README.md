# 원격 처리 (SSH 키 · Linux 데스크톱)

`SETTING.md`에서 분리한 Jetson 원격 접속 문서다. SSH 키 등록과 공통 사항은 여기, 원격 데스크톱은
방식별 문서(VNC · RDP)로 나눈다.

[← SETTING.md](../SETTING.md)

Jetson의 전체 Ubuntu 데스크톱을 원격에서 보려면 SSH 키 등록과 원격 데스크톱 설정을
분리해서 진행한다. SSH 터미널만 필요하면 키 등록까지만 하면 된다.

현재 예시 환경:

```text
PC       : 10.2.12.77
Jetson   : bee.local (10.2.12.110)
Jetson 사용자: bee
```

IP는 DHCP로 바뀔 수 있으므로 가능하면 `bee.local` 또는 고정 IP를 사용한다.

## 방법 고르기

**기본은 VNC다** — 모니터가 있든 없든. 모니터가 없으면 `run-vnc-server --headless`가 X에 가상 모니터를
한 번 꽂아 둔다(소프트웨어 더미 플러그). 어느 쪽이든 GPU로 그린 실제 데스크톱이라 RViz도 쓸 만하다.

| 상황 | 방식 | 문서 | 스크립트 |
| --- | --- | --- | --- |
| **기본** — 모니터 있음, 또는 모니터 없음(`--headless` 한 번) · GPU로 그린 실제 화면 | **VNC** (x11vnc) | [VNC.md](VNC.md) | `run-vnc-server` (Jetson) · `run-vnc-client` (PC) |
| VNC가 안 될 때 — 접속마다 가상 화면 (GPU 없이 CPU로 그려서 느림) | RDP (xrdp) | [RDP.md](RDP.md) | `run-rdp-server` (Jetson, 한 번) · `run-rdp-client` (PC) |
| 위 둘이 안 될 때 | NoMachine | [3장](#3-nomachine-모니터-없이-jetson--pc) | — |
| 터미널만 (operator_tool) | SSH | [2장](#2-ssh-접속과-operator_tool) | — |

```text
run-vnc-server   # Jetson에서 실행 — x11vnc 시작/중지/상태 · 모니터 없으면 --headless 한 번 (기본 방법)
run-vnc-client   # PC에서 실행 — SSH 터널 + Remmina VNC
run-rdp-server   # Jetson에서 한 번 — RDP(xrdp) 설정 (VNC가 안 될 때)
run-rdp-client   # PC에서 실행 — SSH 터널 + Remmina RDP
```

> VNC와 RDP(xrdp)는 함께 쓰지 않는다: `run-rdp-server`가 GDM 자동 로그인을 끄고, 같은 사용자의 GNOME은
> 하나만 뜰 수 있어서 VNC로 공유할 데스크톱이 없어진다. RDP에서 VNC로 돌아오려면 `~/run-rdp-server --undo`.

## 1. PC: SSH 키 등록

PC에 키가 없으면 먼저 만든다.

```bash
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519 -C "bee@jetson"
```

Jetson에 공개키를 등록한다. 이때 Jetson 계정 비밀번호를 한 번 입력한다.

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub bee@bee.local
```

확인:

```bash
ssh -o BatchMode=yes bee@bee.local 'hostname; hostname -I'
```

`Permission denied (publickey,password)`가 나오면 키 등록이 안 된 것이다.
`ssh-copy-id`를 다시 실행하거나 Jetson의 `~/.ssh/authorized_keys`를 확인한다.

매번 긴 주소를 입력하지 않으려면 PC의 `~/.ssh/config`에 추가한다.

```sshconfig
Host bee
  HostName bee.local
  User bee
  IdentitiesOnly yes
  IdentityFile ~/.ssh/id_ed25519
```

이제 다음처럼 접속한다.

```bash
ssh bee
```

## 2. SSH 접속과 operator_tool

`operator_tool`은 TUI이므로 GUI 없이 SSH만으로 동작한다.

```bash
ssh bee@bee.local

cd ~/Lv2_bee_Assignment/lv2_module6/tool
./run-cli
```

RViz 같은 개별 GUI 앱만 필요하면 X11 포워딩을 사용할 수 있다.

```bash
ssh -X bee@bee.local
cd ~/Lv2_bee_Assignment/lv2_module6/tool
./run-rviz
```

X11 포워딩은 전체 데스크톱이 아니라 앱 하나를 PC 화면에 띄우는 방식이다.
RealSense 영상이나 OpenGL 화면은 전달이 느릴 수 있다.

## 3. NoMachine (모니터 없이 Jetson + PC)

VNC(`--headless`)와 RDP가 모두 안 될 때의 대안이다(Jetson에서 아직 미검증). NoMachine은 실제 화면이 없으면
**자기 가상 화면을 만들어** 데스크톱을 띄운다. 소프트웨어 렌더링일 수 있어 RViz 같은 3D는 느릴 수 있다.

### 3.1 설치

nomachine.com → Download → Linux **ARM64(aarch64) DEB**를 받아 Jetson으로 복사한다.

```bash
# PC에서
scp nomachine_*_arm64.deb bee@bee.local:~
# Jetson에서
sudo apt install ./nomachine_*_arm64.deb
systemctl status nxserver --no-pager      # 설치하면 자동 실행
ss -ltnp | grep 4000                       # 4000 포트 확인
```

PC에도 NoMachine(Linux x86_64 DEB 등)을 설치한다.

### 3.2 모니터 없을 때: GDM 끄고 가상 화면 쓰기

모니터 없이 부팅하면 GDM이 출력 없는 X 화면을 띄워 두는 경우가 있다. 이때 NoMachine이
그 화면에 붙어서 검은 화면·로고만 보인다. GDM을 끄면 NoMachine이 자기 가상 화면을 만든다.

```bash
ssh -t bee@bee.local 'sudo systemctl stop gdm && sudo /etc/NX/nxserver --restart'
```

PC NoMachine에서 접속한다.

```text
호스트: bee.local (또는 10.2.12.110)
포트  : 4000 (프로토콜 NX)
사용자: bee / Jetson 계정 비밀번호
```

"Create a new virtual desktop"(또는 새 데스크톱 만들기)가 나오면 선택한다.

가상 화면 해상도·자동 생성은 `/usr/NX/etc/server.cfg`에서 바꿀 수 있다
(주석 `#`을 지우고 값 수정 → `sudo /etc/NX/nxserver --restart`).

```ini
CreateDisplay 1
DisplayOwner "bee"
DisplayGeometry 1280x720
```

### 3.3 부팅할 때부터 GDM 안 띄우기 (잘 되면)

```bash
ssh -t bee@bee.local 'sudo systemctl set-default multi-user.target'
```

모니터를 다시 꽂아 쓸 때는 되돌린다.

```bash
ssh -t bee@bee.local 'sudo systemctl set-default graphical.target && sudo systemctl start gdm'
```

### 3.4 안 될 때

- 검은 화면·NVIDIA 로고만 나옴: `systemctl status gdm`이 아직 active인지 확인 → 3.2
- 접속 자체가 안 됨: `ss -ltnp | grep 4000`, `sudo /etc/NX/nxserver --status`
- 로그: `sudo tail -50 /usr/NX/var/log/nxserver.log`
- 4000 포트를 외부 인터넷에 열지 않는다. LAN 밖에서는 SSH 터널(`ssh -N -L 4000:localhost:4000 bee@bee.local` → `localhost:4000`)

## 4. 원격 GUI와 ROS 주의사항

- `operator_tool`은 SSH 터미널에서 실행하는 것이 가장 가볍다.
- RViz를 Jetson에서 띄우고 x11vnc/RDP/NoMachine으로 화면만 보는 방식이
  Wi-Fi에서 RealSense raw 영상을 직접 전송하지 않아 유리하다.
- PC에서 RViz가 직접 ROS 토픽을 구독하게 하려면 `ROS_LOCALHOST_ONLY`를 끄고
  `ROS_DOMAIN_ID=88`을 맞춰야 한다. 이 경우 카메라/점구름 트래픽이 Wi-Fi를 탄다.
- Jetson 로컬에서만 ROS를 쓰려면 `ROS_LOCALHOST_ONLY=1`을 유지해도 되고,
  원격 데스크톱으로 화면만 보면 된다.

## 5. 공통 체크리스트

```text
[ ] ssh-copy-id로 공개키 등록
[ ] ssh bee@bee.local 접속 성공
[ ] VNC.md 체크리스트 (모니터 없으면 run-vnc-server --headless 한 번)
[ ] (VNC 불가) RDP.md 체크리스트
[ ] (둘 다 불가) NoMachine 설치 → gdm 중지 → PC NoMachine 4000 접속
```

문제가 생기면 먼저 확인할 것:

```bash
# SSH 키
ssh -v bee@bee.local

# 데스크톱 세션
loginctl list-sessions
```
