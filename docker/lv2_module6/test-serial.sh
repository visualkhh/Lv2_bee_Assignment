#!/bin/bash
# 가상 시리얼 브릿지 자가진단 (컨테이너 안에서 실행)
#   test-serial
# file → serial → file 왕복이 되면 PASS, 아니면 FAIL (exit 1)
set -u
SERIAL=/dev/opencr   # 앱 쪽 이름 (실기 udev와 같음 — 컨테이너는 PTY ttyV0을 가리키는 링크)
IN=/debug/input/hardware/serial
OUT=/debug/output/hardware/serial

# 포트 모드 강제 (pyserial 등이 cooked로 바꿔놓고 가면 I/O가 깨짐)
stty -F /dev/opencr raw -echo 2>/dev/null
stty -F /dev/ttyV1 raw -echo 2>/dev/null

[ -e "$SERIAL" ] || { echo "FAIL: $SERIAL 없음 (브릿지 미구동)"; exit 2; }
[ -f "$IN" ] || { echo "FAIL: $IN 없음"; exit 2; }

TOKEN="LOOPBACK-$(date +%s)"
: > "$OUT"

# A: file → serial (input/hardware/serial에 쓰면 /dev/opencr에서 읽혀야 함)
(timeout 12 cat "$SERIAL" > /tmp/serial_test_a &)
sleep 2
echo "$TOKEN-A" >> "$IN"
sleep 9
grep -q "$TOKEN-A" /tmp/serial_test_a 2>/dev/null || {
  echo "FAIL: file→serial (input/hardware/serial 내용이 $SERIAL에 안 옴)"
  exit 1
}
echo "ok: file→serial"

# B: serial → file (/dev/opencr에 쓰면 output/hardware/serial에 쌓여야 함)
echo "$TOKEN-B" > "$SERIAL"
sleep 3
grep -q "$TOKEN-B" "$OUT" || {
  echo "FAIL: serial→file ($SERIAL 내용이 $OUT에 안 쌓임)"
  exit 1
}
echo "ok: serial→file"

rm -f /tmp/serial_test_a
echo "PASS: serial bridge loopback"
