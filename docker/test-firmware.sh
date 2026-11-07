#!/bin/bash
# 펌웨어 검사 (컨테이너 안에서 실행): test-firmware
#   1. FQBN 유효성 (board details — 보드 연결 없이 검증)
#   2. /firmware 아래 전 스케치 컴파일 → /debug/firmware/<스케치>/ (매번 비우고 시작, 호스트 참고용·gitignore)
#   3. arduino-cli upload로 가상 시리얼(/dev/opencr)에 .bin 전송
#      → output/hardware/serial이 .bin과 바이트 단위로 같으면 PASS
# 업로드 recipe만 --upload-property로 바꿔서 opencr_ld 대신 .bin을 포트에 raw로 씀.
# (opencr_ld는 보드 ACK 없으면 핸드셰이크만 보내고 멈춤 + arm64엔 아예 없음)
# 실물 업로드는 실기 PC에서 recipe 덮어쓰기 없이:
#   arduino-cli upload -p /dev/opencr --fqbn OpenCR:OpenCR:OpenCR --input-dir <빌드폴더> <스케치>
set -u
FQBN=OpenCR:OpenCR:OpenCR
SKETCH_DIR=/firmware
BUILD_DIR=/debug/firmware
UPLOAD_PORT=/dev/opencr   # 실기와 같은 이름 (firmware/upload.sh 기본)
SERIAL_OUT=/debug/output/hardware/serial

# 포트 모드 강제 (pyserial 등이 cooked로 바꿔놓고 가면 I/O가 깨짐)
stty -F /dev/opencr raw -echo 2>/dev/null
stty -F /dev/ttyV1 raw -echo 2>/dev/null

arduino-cli board details --fqbn "$FQBN" >/dev/null 2>&1 \
  || { echo "FAIL: FQBN 무효 ($FQBN)"; exit 1; }
echo "ok: FQBN $FQBN"

[ -e "$UPLOAD_PORT" ] || { echo "FAIL: 가상 시리얼 없음 ($UPLOAD_PORT — 브릿지 미구동?)"; exit 2; }

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

found=0
for sketch in "$SKETCH_DIR"/*/; do
  # shellcheck disable=SC2144
  [ -e "${sketch}"*.ino ] || continue
  found=1
  name=$(basename "$sketch")
  out="$BUILD_DIR/$name"

  echo "--- compile: $sketch -> $out"
  arduino-cli compile --fqbn "$FQBN" --library Dynamixel2Arduino --output-dir "$out" "$sketch" \
    || { echo "FAIL: compile $name"; exit 1; }
  bin=$(ls "$out"/*.bin)
  size=$(wc -c < "$bin")

  echo "--- upload: $bin ($size bytes) -> $UPLOAD_PORT"
  : > "$SERIAL_OUT"
  arduino-cli upload -p "$UPLOAD_PORT" --fqbn "$FQBN" --input-dir "$out" \
    --upload-property 'upload.pattern=cp "{build.path}/{build.project_name}.bin" "{serial.port}"' \
    "$sketch" || { echo "FAIL: upload $name"; exit 1; }

  # socat이 파일로 다 흘려보낼 때까지 대기 (최대 10초)
  for _ in $(seq 20); do
    [ "$(wc -c < "$SERIAL_OUT")" -ge "$size" ] && break
    sleep 0.5
  done
  cmp -s "$bin" "$SERIAL_OUT" \
    || { echo "FAIL: upload 바이트 불일치 ($name: bin $size / output/hardware/serial $(wc -c < "$SERIAL_OUT") bytes)"; echo "  다른 프로세스가 시리얼에 쓰는 중? → pgrep -fa dynamixel_controller"; exit 1; }
  echo "ok: $name — output/hardware/serial == .bin ($size bytes)"
done

if [ "$found" = 0 ]; then
  echo "SKIP: 스케치 없음 ($SKETCH_DIR)"
  exit 0
fi
echo "PASS: firmware (compile + upload → 가상 시리얼)"
