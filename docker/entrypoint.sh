#!/bin/bash
set -e
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

# 가상 시리얼 브릿지 (OpenCR 실기 없이 통신 로직 검증용 — module5와 같은 구성)
#   앱(BeebotSystemHardware·업로더) ↔ /dev/opencr (→ ttyV0) ↔ (socat 중계) ↔ /dev/ttyV1 ↔ input/hardware/serial·output/hardware/serial 파일
#   앱은 실기와 같은 이름 /dev/opencr 만 쓴다 — ttyV0·ttyV1은 PTY 쌍의 내부 이름 (V1은 브릿지 쪽)
#   /dev/opencr에 들어온 데이터 → /debug/output/hardware/serial 에 append   (호스트가 보낸 C 명령, 업로드된 .bin)
#   /debug/input/hardware/serial 에 쓴 데이터 → /dev/opencr 로 전송          (OpenCR 응답 S 상태를 흉내)
rm -f /run/testbed-ready
mkdir -p /debug
# 공유 폴더 권한: 컨테이너(root)와 호스트(일반 사용자)가 서로 만든 파일·폴더를 읽고 써야 함.
# 파일은 644여도 상대가 읽기만 하면 되지만, 디렉토리는 양쪽 다 rename(원자적 쓰기)으로 교체하므로 777 유지.
mkdir -p /debug/input /debug/input/hardware/camera /debug/topic /debug/node /debug/pkg /debug/output/topic /debug/service /debug/action \
  /debug/interface /debug/output /debug/output/hardware
chmod -R 777 /debug
if command -v socat >/dev/null 2>&1; then
  # 시작할 때마다 로그 파일 초기화 (빈 파일 재생성)
  : > /debug/input/hardware/serial
  : > /debug/output/hardware/serial
  # 컨테이너(root)가 만든 파일이라 호스트 사용자도 쓸 수 있게 (시뮬레이터가 input/hardware/serial에 OpenCR 응답을 써 넣음)
  chmod 666 /debug/input/hardware/serial /debug/output/hardware/serial
  # 1. PTY 쌍 생성 (/dev/ttyV0 = 앱 쪽, 아래 /dev/opencr 링크로 연다 · /dev/ttyV1 = 브릿지 쪽)
  socat pty,raw,echo=0,link=/dev/ttyV0 pty,raw,echo=0,link=/dev/ttyV1 &
  sleep 1
  # 실기 OpenCR 포트명도 가상 시리얼로 연결 (실물 보드를 넘겨받았으면 그대로 둠)
  #   /dev/opencr = 실기 udev 고정 이름 (URDF port 기본 — BeebotSystemHardware · 펌웨어 업로드 기본)
  [ -e /dev/opencr ] || ln -s /dev/ttyV0 /dev/opencr
  # 2. 시리얼 → 파일 (앱이 보낸 데이터를 output/hardware/serial에 축적)
  socat -u /dev/ttyV1,raw,echo=0 OPEN:/debug/output/hardware/serial,creat,append &
  # 3. 파일 → 시리얼 (input/hardware/serial에 추가된 내용을 앱으로 전송)
  # -u 단방향 필수: 양방향이면 이 프로세스가 /dev/ttyV1 읽기를 나눠가져 output/hardware/serial이 유실됨
  tail -c +1 -F /debug/input/hardware/serial 2>/dev/null | socat -u - /dev/ttyV1,raw,echo=0 &
fi
# 준비 완료 표시: docker run -d 직후 바로 exec하는 쪽(CI)은 이 파일이 생길 때까지 기다린다
touch /run/testbed-ready

exec "$@"
