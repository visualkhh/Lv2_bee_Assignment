# Jetson 개발 접속 (Discovery Server)

같은 `ROS_DOMAIN_ID`인데 PC에서 토픽·노드가 덜 보이면, 공유기가 RTPS
discovery 멀티캐스트(`239.255.0.1`)를 삼키는 경우다. 이럴 땐 Discovery
Server로 유니캐스트 discovery를 쓴다.

원격 데스크톱·SSH 자체는 [`remote/README.md`](../remote/README.md) 참고.
여기는 ROS 통신 discovery만 다룬다.

## 방식 확정: 서버는 Jetson, PC가 붙는다

`ROS_DISCOVERY_SERVER`에 적는 건 **서버에 닿는 주소**다. 같은 서버를
가리키기만 하면 기계마다 달라도 된다:

```text
Jetson 스택: ROS_DISCOVERY_SERVER="127.0.0.1:11811"   # 자기 자신이라 lo로
PC:          ROS_DISCOVERY_SERVER="JetsonIP:11811"     # LAN으로 (지금 10.2.12.110)
```

Jetson은 자기 자신을 보니까 자기 IP가 바뀌어도 스택 설정을 안 고쳐도 된다.

## ROS_STATIC_PEERS란 (여기서는 안 씀)

discovery 서버 없이, “저 주소에 참가자가 있으니 직접 찾아가라”고 찍어주는
옛날 방식이다. seed 1곳에만 보내서 전체 mesh가 안 만들어지고, 지금처럼
일부만 보이는 원인이 된다. 서버 방식을 쓰면 필요 없으니 `unset`한다.

## 1. 서버 띄우기 (Jetson에서, 1개만)

Jetson에 SSH로 들어가서:

```bash
source /opt/ros/lyrical/setup.bash
setsid fastdds discovery -i 0 -p 11811 > /tmp/ds.log 2>&1 < /dev/null &
ss -uln | grep 11811
```

## 2. Jetson 스택 (재시작 필요)

`lv2_module6/ros2_ws`에서:

```bash
export ROS_DOMAIN_ID=99
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
./start.sh
```

`ROS_DISCOVERY_SERVER`가 localhost·자기 IP면 start.sh가 서버도 같이 띄운다.
원격 주소면 붙기만 한다. 미설정이면 멀티캐스트 기본.

## 3. PC 터미널

```bash
export ROS_DOMAIN_ID=99
export ROS_DISCOVERY_SERVER="10.2.12.110:11811"
unset ROS_STATIC_PEERS
ros2 daemon stop
ros2 topic list | wc -l
```

## IP 확인 (PC가 쓸 Jetson 주소 — DHCP라 바뀔 수 있음)

PC가 넣는 건 **Jetson의 IP**다. Jetson에서 확인:

```bash
# Jetson에서: PC로 가는 자신의 주소 = PC가 서버 주소로 쓸 값
ip route get 10.2.12.77 | grep -oP 'src \K\S+'
# 지금은 10.2.12.110 나옴 → PC는 ROS_DISCOVERY_SERVER="10.2.12.110:11811"
```

참고로 PC 자신의 IP가 필요할 일은 없다 (서버에 붙는 쪽이라 주소 등록이 자동).
굳이 확인하려면 PC에서:

```bash
ip route get 10.2.12.110 | grep -oP 'src \K\S+'
```

`ROS_DISCOVERY_SERVER`에는 `bee.local` 같은 호스트명 말고 **IP 숫자**를 넣는다.

## 주의

- 서버가 죽으면 신규 참가자를 못 찾는다 (이미 연결된 건 유지).
- `ROS_STATIC_PEERS`와 병행하지 말 것.
