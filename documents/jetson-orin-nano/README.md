# Jetson Orin Nano 문서

Jetson Orin Nano 실기 세팅·점검·원격 접속 문서를 모아 둔 폴더다.

## 문서

- [`SETTING.md`](SETTING.md)
  - Jetson OS/SDK, **8GB swap(SLAM/mapping 대비)**, ROS 2 Lyrical, RealSense, SLAM(cuVSLAM + nvblox) 설치 안내 → CUVSLAM.md
  - OpenCR/Dynamixel, CI 배포 패키지, 실행 관련 메모
- [`CUVSLAM.md`](CUVSLAM.md)
  - cuVSLAM 소스 빌드(Orin은 배포 휠 불가), SLAM = cuVSLAM(위치) + nvblox(지도) 구성 · nvblox 설치 방법 · 실기 시험 순서, 원격 RViz 주의, 실기 시험 기록
- [`ONNXRUNTIME.md`](ONNXRUNTIME.md)
  - RL 주행 정책(`rl_nav_controller`) ONNX Runtime — CUDA EP 있으면 GPU, 없으면 CPU 자동 선택. Jetson Orin `sm_87` CUDA 13.2 빌드·설치, `ML_ONNXRUNTIME_ROOT`, 도커 GPU 설정
- [`DOCTOR.md`](DOCTOR.md)
  - 커널/arch, JetPack/L4T, glibc/Python, Jetson 모델 기준값
- [`remote/README.md`](remote/README.md)
  - SSH 키, x11vnc/원격 데스크톱, Jetson용 `server.sh`와 PC용 `client.sh`
- [`develop/README.md`](develop/README.md)
  - Discovery Server (멀티캐스트 막힘 우회), 개발 접속용 ROS 환경변수
- [`../ros/DEBUG.md`](../ros/DEBUG.md)
  - `start.sh`/launch 로그, ROS 그래프/QoS, 컨트롤러·라이프사이클, 빌드·기동 에러

## 권장 순서

1. [`SETTING.md`](SETTING.md)로 OS/ROS/센서/매핑 스택을 설치한다.
2. [`DOCTOR.md`](DOCTOR.md)로 현재 Jetson 기준값을 확인한다.
3. 실행 중 문제가 생기면 [`../ros/DEBUG.md`](../ros/DEBUG.md) 절차로 로그를 본다.
4. 원격 작업은 [`remote/README.md`](remote/README.md)와 스크립트를 사용한다.
5. PC에서 토픽이 덜 보이면(멀티캐스트 막힘) [`develop/README.md`](develop/README.md)의 Discovery Server를 쓴다.
