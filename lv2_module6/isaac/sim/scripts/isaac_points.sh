#!/usr/bin/env bash
# Isaac Sim 카메라 depth → PointCloud2 (depth_image_proc)
#
# Isaac Sim 6.1 + Lyrical에서 ROS2 Camera Helper `depth_pcl`은 Play 직후 죽는다 (segfault).
# Lyrical용 브리지 백엔드가 없어 Jazzy 백엔드(libisaacsim.ros2.core.jazzy.so)가 대신 돌고,
# PointCloud2 직렬화(get_serialized_size_sensor_msgs__msg__PointCloud2)에서 터진다. Image·CameraInfo·Imu는 정상.
# → Isaac은 정렬 depth·camera_info까지만 내고, 점구름은 여기서 만든다.
#
#   sudo apt install ros-lyrical-depth-image-proc
#   isaac/sim/scripts/isaac_points.sh        # Isaac Sim을 띄우고 점구름 노드를 같이 돌린다. Isaac을 끄면 노드도 꺼진다
#   ISAACSIM_VENV=<venv 경로> isaac/sim/scripts/isaac_points.sh   # venv 위치가 다를 때 (기본 ~/pa-course/.venv-isaacsim)
# Isaac이 뜨면 usda/beebot/beebot.usda를 열고 Play. 노드는 lazy 구독이라 points 구독자가 붙어야 입력을 받는다.
#
# 입력: /isaac/pole_front_cam/aligned_depth_to_color/image_raw (32FC1 m) · color/camera_info
# 출력: /isaac/pole_front_cam/depth/color/points (m) — 실물 realsense 점구름과 같은 이름·단위
# gripper_cam 점구름은 실물에서도 기본으로 꺼져 있어 만들지 않는다 (ros_system gripper_cam_pointcloud:=false).
source /opt/ros/lyrical/setup.bash   # Isaac도 이 환경(ROS_DISTRO=lyrical, ROS_DOMAIN_ID)을 물려받는다
source "${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/bin/activate"
isaacsim &
ISAAC_PID=$!
NS=/isaac/pole_front_cam
D=$NS/aligned_depth_to_color/image_raw
I=$NS/color/camera_info
# image_transport는 camera_info를 영상 토픽 옆(aligned_depth_to_color/camera_info)에서 찾는다 — color 것으로 돌린다 (같은 RenderProduct라 K 동일)
DI=$NS/aligned_depth_to_color/camera_info
# Isaac 영상은 best_effort(SENSOR_DATA)로 나간다 — reliable로 구독하면 연결되지 않는다
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r __node:=pole_front_cam_points \
  -r image_rect:=$D -r $DI:=$I -r points:=$NS/depth/color/points \
  -p use_sim_time:=true \
  -p qos_overrides.$D.subscription.reliability:=best_effort \
  -p qos_overrides.$I.subscription.reliability:=best_effort \
  -p qos_overrides.$DI.subscription.reliability:=best_effort &
POINTS_PID=$!
trap 'kill $POINTS_PID $ISAAC_PID 2>/dev/null' EXIT INT TERM
wait $ISAAC_PID
