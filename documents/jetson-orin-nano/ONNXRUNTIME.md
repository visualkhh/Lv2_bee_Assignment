# ONNX Runtime (RL 주행 정책) — GPU 자동, 없으면 CPU

`rl_nav_controller`는 `models/rl_nav_controller/policy.onnx`를 ONNX Runtime으로 추론한다.
코드는 **CUDA EP가 있으면 GPU, 없으면 CPU**로 자동 선택한다. 같은 바이너리가 테스트베드(GPU 없음)와
Jetson(GPU 있음)에서 모두 돌아간다.

- 코드: `lv2_module6/ros2_ws/src/ml/src/onnx_policy.cpp`
- 선택 순서: `Ort::GetAvailableProviders()`에 `CUDAExecutionProvider`가 있으면 CUDA 세션 시도 →
  실패(드라이버/커널/provider 없음)하면 CPU 세션으로 폴백
- 확인 로그: `ONNX loaded: ... device=CUDA` 또는 `device=CPU`
- CUDA 시도가 실패하면 `ONNX CUDA EP 실패 — CPU로 폴백: <이유>` 경고가 남는다.

> **중요:** `ros-lyrical-onnxruntime-vendor`는 **CPU 전용**이다.
> 이 패키지를 설치해도 GPU가 있으면 자동으로 GPU가 되지 않는다. GPU를 쓰려면 CUDA EP가 들어간
> ONNX Runtime을 따로 빌드해서 `ml` 패키지가 그쪽을 보게 해야 한다.

---

## 1. 현재 상태 확인

Jetson에서 먼저 확인한다.

```bash
nvcc --version
ls -l /usr/local/cuda/lib64/libcudart.so*
ls -l /usr/lib/aarch64-linux-gnu/libcudnn.so*
```

- JetPack 7.2 / L4T r39.2 / CUDA 13.2 / cuDNN 9 / TensorRT 10.16 기준이다.
- `nvcc`가 없으면 Jetson apt의 CUDA 툴킷을 설치한다 (JetPack SDK에 포함).

기존 ONNX Runtime이 CPU 전용인지 확인:

```bash
ldd /opt/ros/lyrical/opt/onnxruntime_vendor/lib/libonnxruntime.so | grep -E 'cuda|cudnn' || echo 'CPU-only'
ls /opt/ros/lyrical/opt/onnxruntime_vendor/lib/libonnxruntime_providers_cuda.so 2>/dev/null || echo 'CUDA provider 없음'
```

- `libonnxruntime_providers_cuda.so`가 없으면 CPU 전용이다.
- 현재 테스트베드 컨테이너가 이 상태다.

---

## 2. 설치할 것

| 항목 | 용도 | 확인 |
|---|---|---|
| JetPack 7.2 CUDA Toolkit 13.2 | ONNX Runtime CUDA EP 빌드·실행 | `nvcc --version` |
| cuDNN 9 (CUDA 13용) | CUDA EP 실행 | `ls /usr/lib/aarch64-linux-gnu/libcudnn.so*` |
| TensorRT 10.16 | 선택 — 지금 코드는 CUDA EP만 씀 | `dpkg -l | grep tensorrt` |
| build-essential, cmake, ninja, git | ONNX Runtime 소스 빌드 | `cmake --version` |
| python3-dev, python3-pip, python3-setuptools, python3-wheel | ORT 빌드 스크립트 | `python3 --version` |
| libopenblas-dev | ORT CPU 커널 의존 | `dpkg -l | grep openblas` |
| ONNX Runtime 1.24.3 (CUDA EP, sm_87) | `ml` 플러그인이 링크 | `/opt/onnxruntime/lib/libonnxruntime.so` |

설치:

```bash
sudo apt update
sudo apt install -y --no-install-recommends \
    build-essential cmake ninja-build git \
    python3-dev python3-pip python3-setuptools python3-wheel \
    libopenblas-dev
```

cuDNN은 JetPack 기본 설치에 **들어 있지 않을 수 있다** (bee는 없었음). 위 `ls`에 아무것도 안 나오면 설치한다.

```bash
apt-cache policy cudnn9-cuda-13        # Candidate가 보여야 함 (NVIDIA Jetson r39.2 repo)
sudo apt install -y cudnn9-cuda-13 libcudnn9-dev-cuda-13
dpkg -L libcudnn9-dev-cuda-13 | grep -E 'cudnn\.h$|libcudnn\.so$'   # 헤더·라이브러리 위치 확인
```

CUDA 라이브러리 중 **cuFFT도 JetPack 기본에 빠져 있을 수 있다** (bee는 없었음). 없으면 3절 설정 단계에서
`Target "onnxruntime_providers_cuda" links to: CUDA::cufft but the target was not found`로 실패한다.

```bash
ls /usr/local/cuda/targets/*/lib | grep cufft || sudo apt install -y libcufft-dev-13-2
```

- 3절 `--cudnn_home /usr/lib/aarch64-linux-gnu` 그대로 된다 (bee에서 확인). CMake가
  `cudnn found at /usr/lib/aarch64-linux-gnu/libcudnn.so`, 헤더는 `/usr/include/aarch64-linux-gnu`로 알아서 찾는다.

> **TensorRT는 지금 필요 없다.** `onnx_policy.cpp`는 CUDA EP만 등록한다. TensorRT EP까지 쓰려면
> `--use_tensorrt`로 ORT를 빌드하고 코드에 TensorRT EP 등록을 추가해야 한다. 이 MLP에는 이득이 거의 없다.

---

## 3. ONNX Runtime GPU 빌드 (aarch64 · sm_87)

Jetson Orin은 `sm_87`이다. x86_64용 배포 바이너리나 `sm_90`(Grace)·`sm_121`(Thor)용 휠은
`cudaErrorNoKernelImageForDevice`로 실패한다. **소스 빌드가 필요하다.**

```bash
cd ~
git clone --recursive https://github.com/microsoft/onnxruntime
cd onnxruntime
git checkout v1.24.3
git submodule update --init --recursive

export PATH=/usr/local/cuda/bin:$PATH
export CUDACXX=/usr/local/cuda/bin/nvcc

./build.sh --config Release --update --build --parallel 4 \
  --build_shared_lib --skip_tests \
  --use_cuda \
  --cuda_home /usr/local/cuda \
  --cudnn_home /usr/lib/aarch64-linux-gnu \
  --cmake_extra_defines CMAKE_CUDA_ARCHITECTURES=87 \
  --compile_no_warning_as_error
```

- `--build_shared_lib`가 **C++ 플러그인이 링크할 `libonnxruntime.so`** 를 만든다. Python 휠만으로는 부족하다.
- `CMAKE_CUDA_ARCHITECTURES=87`이 핵심이다. 이 값이 없거나 다른 값이면 provider 목록에는 CUDA가 보여도
  실제 커널이 없어 런타임에 실패한다.
- `--compile_no_warning_as_error` — CUDA 13에서 ORT 1.24의 deprecation 경고를 에러로 승격하지 않게 한다.
- ORT 1.24.1부터 CUDA 13.0을 지원한다. CUDA 13.2는 그 상위 버전이라 호환된다.
- Orin Nano 8GB에서는 `--parallel 4`가 빠듯할 수 있다. `killed`가 나오면 `--parallel 2`로 줄인다.
  빌드는 수 시간 걸리고 메모리를 많이 쓴다. swap 8GB로 부족하면 16GB로 늘린다 (SETTING.md의 swap 절 참고).
- 설정 단계에서 실패해 의존성을 추가 설치한 뒤에는 `rm -f build/Linux/Release/CMakeCache.txt`만 지우고
  같은 명령을 다시 실행한다 (받아 둔 `_deps`는 유지돼 설정이 빨리 끝난다).
- 중간에 OOM이 나면 `rm -rf build/Linux/Release` 후 `--parallel 2`로 다시 실행한다.
- **빌드가 중간에 끊겼을 때(재부팅·먹통)**: `.o`가 남아 있으니 `--update` 없이 `--build`만 줘서 이어 간다.
  처음부터 다시 할 필요 없다.
  `./build.sh --config Release --build --parallel 2 --build_shared_lib --skip_tests --use_cuda --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu --cmake_extra_defines CMAKE_CUDA_ARCHITECTURES=87 --compile_no_warning_as_error`
- **빌드 중에는 다른 무거운 작업(`colcon build` 등)을 같이 돌리지 않는다.** RAM 7.4GB에서 `cicc`(CUDA 컴파일러)
  프로세스 하나가 약 2.6GB를 쓴다. 겹치면 swap을 쓰다 SSH까지 먹통이 된다 (아래 삽질 기록).
- 끝났는지는 `ls build/Linux/Release/libonnxruntime_providers_cuda.so`로 본다. 진행 중이면
  `ps -eo pid,etime,pcpu,args | grep -E 'cicc|ptxas' | grep -v grep`에 컴파일러가 보인다.
  flash attention 커널(`flash_fwd_*_sm80.cu.o`)이 오래 걸린다. bee에서 정지처럼 보여도 컴파일 중인 경우가 많다.

### 3-1. `/opt/onnxruntime`에 설치

빌드가 끝나면 라이브러리와 헤더를 한 곳에 모은다.

```bash
cd ~/onnxruntime
sudo mkdir -p /opt/onnxruntime
sudo cmake --install build/Linux/Release --prefix /opt/onnxruntime
# 설치 규칙이 provider .so를 빼먹는 경우 대비 — 같은 파일을 한 번 더 복사 (없으면 무시)
sudo cp -a build/Linux/Release/libonnxruntime*.so* /opt/onnxruntime/lib/ 2>/dev/null || true
sudo find include/onnxruntime -name '*.h' -exec cp {} /opt/onnxruntime/include/ \; 2>/dev/null || true
```

이 명령은 보통 다음을 설치한다.

```text
/opt/onnxruntime/include/onnxruntime/onnxruntime_cxx_api.h
/opt/onnxruntime/lib/libonnxruntime.so -> libonnxruntime.so.1.24.3
/opt/onnxruntime/lib/libonnxruntime_providers_cuda.so
/opt/onnxruntime/lib/libonnxruntime_providers_shared.so
```

확인:

```bash
ls -l /opt/onnxruntime/lib | grep -E 'onnxruntime(\.so|_providers)'
ldd /opt/onnxruntime/lib/libonnxruntime_providers_cuda.so | grep -E 'cudart|cudnn'
```

`libonnxruntime_providers_cuda.so`가 없으면 `--use_cuda`가 빠졌거나 빌드가 실패한 것이다.
`cudart`/`cudnn`이 `not found`면 4절의 `ldconfig`를 먼저 한다.

---

## 4. 런타임 라이브러리 경로

`ml` 플러그인은 `/opt/onnxruntime/lib`을 RPATH로 갖고 빌드된다. CUDA/cuDNN은 시스템 경로에서 찾는다.
한 번만 설정하면 된다.

> **`ldconfig`만으로는 부족하다.** ROS `setup.bash`가 `LD_LIBRARY_PATH`에 vendor(CPU 전용) 경로를 넣고,
> vendor도 같은 이름(`libonnxruntime.so.1`)이라 `LD_LIBRARY_PATH`가 RPATH·ldconfig보다 먼저 이긴다.
> 그래서 `ml`을 `/opt/onnxruntime`으로 빌드해도 실행 시에는 vendor가 로드되어 `device=CPU`가 된다.
> `start.sh`는 `/opt/onnxruntime`에 CUDA provider가 있으면 `LD_LIBRARY_PATH` 맨 앞에 자동으로 붙인다.
> 직접 실행할 때는 `LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH`를 앞에 붙인다.

```bash
sudo tee /etc/ld.so.conf.d/onnxruntime.conf >/dev/null <<'EOF'
/opt/onnxruntime/lib
/usr/local/cuda/lib64
EOF
sudo ldconfig
```

확인:

```bash
ldconfig -p | grep -E 'onnxruntime|cudart|cudnn' | head
```

> `cmake --install`이 헤더를 `include/onnxruntime/` 아래에 두므로, `ml`의 CMake는
> `include/`와 `include/onnxruntime/` 둘 다 찾는다.

---

## 5. 워크스페이스 빌드

`ml` 패키지가 vendor(CPU) 대신 `/opt/onnxruntime`을 보게 한다.

```bash
cd <repo>/lv2_module6/ros2_ws
source /opt/ros/lyrical/setup.bash

colcon build --symlink-install --packages-select ml \
  --cmake-args -DML_ONNXRUNTIME_ROOT=/opt/onnxruntime \
  --event-handlers status- console_cohesion+
```

- `already built in one or more underlay workspaces` 경고(`'ml' is in: /home/<다른 계정>/...`)는 에러가 아니다.
  셸에 다른 계정의 워크스페이스가 `source`돼 있다는 뜻이다. 새 셸에서 `/opt/ros/lyrical/setup.bash`만 source하면 없어진다.
- colcon이 빌드 도중 `TypeError: can't multiply sequence by non-int of type 'NoneType'`로 멈추면
  진행 표시줄(`colcon_notification`) 버그다. 빌드 실패가 아니다. 아래 `--event-handlers`를 붙이면 피한다.
- `ML_ONNXRUNTIME_ROOT`는 CMake cache 변수다. 한 번 넣으면 이후 `colcon build`에서도 유지된다.
- `build/ml` 캐시가 vendor를 보고 있으면 한 번 지우고 다시 빌드한다.

```bash
rm -rf build/ml install/ml
colcon build --symlink-install --packages-select ml \
  --cmake-args -DML_ONNXRUNTIME_ROOT=/opt/onnxruntime
```

- 컨테이너에서 빌드할 때는 `build-ws`를 쓴다.

```bash
build-ws --packages-select ml --cmake-args -DML_ONNXRUNTIME_ROOT=/opt/onnxruntime
```

---

## 6. 확인

### 6-1. 단위 테스트 (가장 빠름)

`test_nav_policy_onnx`가 실제 세션의 device를 출력한다.

```bash
cd <repo>/lv2_module6/ros2_ws
# vendor(CPU)보다 CUDA ORT가 먼저 로드되게 (start.sh는 이걸 자동으로 함)
LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH ./build/ml/test_nav_policy_onnx
```

정상 예:

```text
[onnx] device=CUDA (CUDA EP 있으면 CUDA, 없으면 CPU)
[  PASSED  ] 2 tests.
```

GPU가 없거나 CUDA EP가 없으면:

```text
[onnx] device=CPU (CUDA EP 있으면 CUDA, 없으면 CPU)
```

둘 다 정상이다. **GPU가 있는데 `device=CPU`면** 3~4절을 다시 확인한다.
CUDA를 시도했다가 실패했으면 같은 줄 끝에 `· CUDA 실패: <이유>`가 붙는다.

> **`device=CPU`인데 CUDA ORT를 설치했다면:** ROS vendor의 `libonnxruntime.so.1`(CPU 전용)이 `LD_LIBRARY_PATH`로
> 먼저 잡힌 것이다 (`ldd build/ml/test_nav_policy_onnx | grep onnxruntime`이 `/opt/ros/...`를 가리킴).
> 단독 실행은 `LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH ./build/ml/test_nav_policy_onnx`.
> `start.sh`는 `/opt/onnxruntime`에 CUDA provider가 있으면 자동으로 앞에 붙인다.

### 6-2. 실제 노드 로그

```bash
./start.sh map:=saved 2>&1 | grep -E 'ONNX loaded|CUDA EP|RlNavController'
```

정상 예:

```text
[controller_server] ONNX loaded: .../policy.onnx (obs 375 → 2 · device=CUDA)
[controller_server] RlNavController: inner=... · model=rl_nav_controller/policy.onnx · device=CUDA ...
```

폴백이면:

```text
[controller_server] ONNX CUDA EP 실패 — CPU로 폴백: <이유>
[controller_server] ONNX loaded: ... device=CPU
```

`<이유>`에 `Failed to load library ... libonnxruntime_providers_cuda.so`가 나오면
`/opt/onnxruntime/lib`에 provider가 없거나 `ldconfig`가 안 된 것이다.

### 6-3. CLI에서 보기

플러그인이 1 Hz로 `/diagnostics`에 상태를 낸다. `diagnostic_aggregator`가 `/diagnostics_agg`로 묶고,
CLI가 그걸 읽는다. 별도 서비스를 만들 필요 없다.

```bash
# 실기
./run-cli status --diagnostics controller_server
# 도커 테스트베드
cd lv2_module6/tools && ./run-cli --source docker status --diagnostics controller_server
```

출력 예:

```text
[제공 패키지 / controller_server: FollowPath]
 레벨: OK
 메시지: idle · enable=on · CUDA · ONNX Runtime 1.24.3 · fallback=DWB
 값:
  policy      loaded
  enable      true
  active      false
  mode        idle
  device      CUDA
  provider    CUDAExecutionProvider
  runtime     ONNX Runtime 1.24.3
  model       .../policy.onnx
  inner       DWB
  sensor      ok
  sensor_age  0.05
  scan_min    0.42
  reason      idle
  cmd         v=0.00 w=0.00
```

- `policy=loaded` — 모델이 메모리에 올라옴 (`none`이면 모델 로드 실패)
- `enable=true|false` — `FollowPath.enable` 파라미터. `false`면 정책을 안 씀
- `active=true|false` — 지금 실제로 RL 정책이 명령을 내는 중인지. `false`면 `idle`이거나 `inner`(fallback) 사용
- `mode=RL|idle|DWB|MPPI|…` — 현재 모드
- `device=CUDA|CPU` — 실제 ONNX 세션이 쓰는 EP
- `provider=CUDAExecutionProvider|CPUExecutionProvider`
- `runtime=ONNX Runtime <버전>`
- `model=.../policy.onnx` — 실제로 연 ONNX 파일 경로
- `inner=DWB|MPPI|RPP|RotationShim` — ONNX를 안 쓸 때 넘기는 기본 제어기
- `reason` — RL / enable=false / 센서 없음/stale / 추론 실패 / idle

CLI 메뉴에서는 **노드 → `controller_server` → 상태**와 **A/B → `rl_nav_controller`** 화면에도
`device · provider · runtime` 줄이 나온다.

`FollowPath.enable` 파라미터는 **바꾸는 즉시 적용**된다 (플러그인의 `add_on_set_parameters_callback`).
`/controller_server/FollowPath/toggle` 서비스를 부르지 않아도 `false`면 DWB 바이패스, `true`면 정책으로
바뀐다. 둘은 같은 상태(`enable_`)를 바꾼다. 모델이 없는데 `true`로 켜면 파라미터 변경이 거절되고
이유가 응답에 남는다.

### 6-4. provider 목록 직접 확인

`GetAvailableProviders()`는 **컴파일된 목록**이라, 잘못된 arch 빌드도 CUDA를 보여 줄 수 있다.
실제 커널이 도는지는 6-1의 추론 실행으로 확인한다.

```bash
cat >/tmp/ort_providers.cpp <<'EOF'
#include <onnxruntime_cxx_api.h>
#include <iostream>
int main() {
  for (auto & p : Ort::GetAvailableProviders()) std::cout << p << "\n";
}
EOF
g++ -std=c++17 /tmp/ort_providers.cpp \
  -I/opt/onnxruntime/include/onnxruntime \
  -L/opt/onnxruntime/lib -lonnxruntime \
  -Wl,-rpath,/opt/onnxruntime/lib -o /tmp/ort_providers
/tmp/ort_providers
```

정상 예:

```text
CUDAExecutionProvider
CPUExecutionProvider
```

---

## 7. 도커에서 쓸 때 (Jetson)

현재 `docker/lv2_module6/Dockerfile`은 `ubuntu:24.04` 베이스라 CUDA/cuDNN이 없다.
컨테이너에서 GPU ONNX Runtime을 쓰려면 둘 중 하나가 필요하다.

1. **JetPack CUDA 베이스 이미지로 빌드** — `nvcr.io/nvidia/l4t-jetpack:r39.2.0` 같은
   JetPack 7.2용 베이스에 이 저장소의 Dockerfile 단계를 얹는다.
2. **호스트에서 GPU ORT를 빌드하고 컨테이너에 마운트** — 컨테이너에 CUDA/cuDNN 런타임이 있어야
   `libonnxruntime.so`가 로드된다. `ubuntu:24.04` 베이스만으로는 부족하다.

`docker-compose.yml`에 GPU 접근과 ORT 마운트를 추가하는 예:

```yaml
services:
  lyrical:
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility
      - LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64
    volumes:
      - /opt/onnxruntime:/opt/onnxruntime:ro
```

- `runtime: nvidia`는 Jetson의 NVIDIA Container Toolkit이 설치돼 있어야 한다.
- 컨테이너 안에 CUDA/cuDNN이 없으면 `libonnxruntime_providers_cuda.so`가 로드되지 않는다.
  이때는 CPU로 폴백하므로 노드는 죽지 않지만 GPU는 못 쓴다.
- 컨테이너 빌드 시 `build-ws --packages-select ml --cmake-args -DML_ONNXRUNTIME_ROOT=/opt/onnxruntime`.

실기기에서 하드웨어(OpenCR·RealSense)를 직접 쓰는 경우에는 **호스트 네이티브 실행이 더 단순하다.**
이 문서의 1~6절은 호스트 기준이다.

---

## 8. 성능 메모

- 이 정책은 입력 375 float, 출력 2 float의 **초소형 MLP**다 (policy.onnx 7 KB + 데이터 259 KB).
- CPU 1스레드 추론이 이미 수 ms 수준이다. GPU 전송·동기화 오버헤드가 더 클 수 있다.
- Jetson GPU는 cuVSLAM·nvblox·YOLO가 쓰는 게 효율적이다. RL 정책은 CPU로 두고,
  실제 주행 주기(20 Hz)에서 병목이 확인될 때만 GPU를 켠다.
- GPU를 켜고 끄는 것은 `controller_server`의 `FollowPath.enable`(A/B 토글)로 한다.
  ONNX device는 프로세스 시작 시 정해지고, `device=CPU`로 폴백해도 주행 자체는 계속된다.

---

## 9. 삽질 기록 (bee · Orin Nano 8GB, 2026-11-09)

처음 설치하며 겪은 순서대로 적는다. 같은 데서 막히면 이 목록부터 본다.

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| 1 | `ml` 빌드: `ML_ONNXRUNTIME_ROOT=/opt/onnxruntime — include/onnxruntime_cxx_api.h 또는 lib/libonnxruntime.so가 없음` | `/opt/onnxruntime`이 빈 폴더. ORT를 빌드·설치하기 전에 `ml`부터 빌드했다 | 3절(빌드) → 3-1절(설치) → 그다음 `ml`. 순서를 지킨다 |
| 2 | `build.sh`가 어디 있는지 모름 | `~/onnxruntime`(소스 clone 폴더)에 있다. clone을 안 한 상태였다 | 3절의 `git clone` 부터. `ls ~/onnxruntime/build.sh` |
| 3 | 설정 단계 실패: `Target "onnxruntime_providers_cuda" links to: CUDA::cufft but the target was not found` | JetPack에 `libcufft`가 안 들어 있다 | `sudo apt install -y libcufft-dev-13-2`, `CMakeCache.txt`만 지우고 재실행 |
| 4 | cuDNN 없음 | JetPack 기본에 없고 apt에서 따로 받아야 한다 | `sudo apt install -y cudnn9-cuda-13 libcudnn9-dev-cuda-13` |
| 5 | 빌드 중 bee가 먹통, SSH `No route to host`, 핑 100% 손실 | ORT CUDA 빌드가 RAM을 거의 다 쓰는 중(`cicc` 하나가 약 2.6GB, 여유 79MB)에 `colcon build`까지 같이 돌렸다 | 재부팅. `--build`만 줘서 이어 빌드(`--parallel 2`). 빌드 중에는 다른 무거운 작업 금지 |
| 6 | colcon이 `TypeError: can't multiply sequence by non-int of type 'NoneType'`로 멈추고 `7 packages not processed` | `colcon_notification` 진행 표시줄 버그 | `--event-handlers status- console_cohesion+` |
| 7 | `ml` 빌드는 성공인데 테스트가 `device=CPU` | 실행 시 ROS vendor의 `libonnxruntime.so.1`(CPU)이 `LD_LIBRARY_PATH` 때문에 먼저 로드됨. `ldd`가 `/opt/ros/lyrical/opt/onnxruntime_vendor/...`를 가리킨다 | `LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH`. `start.sh`는 자동으로 붙인다 |
| 8 | 7번을 풀어도 CPU로 폴백, 이유가 `Attempt to use DefaultLogger but none has been registered` | 코드 버그. `Ort::Env`가 세션을 만들 때(`std::make_shared<Ort::Session>(env(), ...)`)에야 생성되는데, CUDA provider는 `AppendExecutionProvider_CUDA`에서 로드되며 로거를 쓴다 | `onnx_policy.cpp`에서 CUDA 시도 직전에 `env();`를 먼저 호출 (커밋 `347e287`). 테스트가 `· CUDA 실패: <이유>`를 출력하도록 함 |
| 9 | `already built in ... underlay workspaces: 'ml' is in: /home/<다른 계정>/...` 경고 | 셸에 다른 계정 워크스페이스가 source돼 있다 | 에러 아님. 새 셸에서 `/opt/ros/lyrical/setup.bash`만 source |
| 10 | 같은 문서 명령을 `/home/bee/...`와 `/home/khh/...` 두 체크아웃에서 섞어 실행 | 체크아웃마다 `build/ml`이 따로다 | 하나로 정해서 그 폴더에서만 빌드·실행 |

**로그에 보이지만 무시해도 되는 것**

- `GPU device discovery failed ... /sys/class/drm/card1/device/vendor`: ORT가 GPU 목록을 sysfs에서 읽는 부가 경고다. Jetson에는 그 파일이 없는 게 정상이다.
- `F16C/FMA/AVX instruction set is not supported`: aarch64라서 정상이다.
- `Excluding SM 80/86/89/...`: ORT가 `CMAKE_CUDA_ARCHITECTURES=87`만 남긴다는 뜻이다.
- `CUDA_CUDA_LIBRARY ... not used`: CMake 경고일 뿐이다.
- `get_package_share_directory is deprecated` (`rl_nav_controller.cpp`): 빌드는 정상이다.

**결과**

- 소스 빌드는 `/home/bee/onnxruntime/build/Linux/Release`에서 했다. 재부팅 전후로 나뉘어 총 수 시간 걸렸다 (정확한 시간은 기록하지 않았다).
- 설치 후 `test_nav_policy_onnx`가 `device=CUDA`, 2 tests PASSED.
- 노드(`controller_server`)가 `device=CUDA`로 뜨는지는 `start.sh` 로그의 `ONNX loaded: ... device=CUDA`로 따로 확인해야 한다.
