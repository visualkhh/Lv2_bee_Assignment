cmake_minimum_required(VERSION 3.16)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# NOTE (Lyrical): ament_target_dependencies 매크로가 삭제됨 → target_link_libraries 사용.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)

# YOLO 노드 (Python, scripts/perception_node.py) — 메시지 패키지는 이 CMake가 만든다
install(PROGRAMS scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node)

# 파라미터 (bringup이 share/perception/config/perception.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# 학습 가중치 — 워크스페이스 공용 lv2_module6/ros2_ws/model/yolo/*.pt 를 share에 설치
# (urdf와 같은 패턴: ROS 밖 train_yolo.py가 쓰는 파일을 패키지가 가져옴)
set(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../model/yolo" CACHE PATH "ros2_ws/model/yolo")
file(GLOB MODEL_FILES "${MODEL_DIR}/*.pt")
if(MODEL_FILES)
  install(FILES ${MODEL_FILES} DESTINATION share/${PROJECT_NAME}/model)
else()
  message(WARNING "YOLO 가중치 없음 (${MODEL_DIR}/*.pt) — 노드는 빈 목록 발행")
endif()

# --- 테스트: colcon test --packages-select perception
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node test/test_node.py TIMEOUT 120 ENV ROS_DOMAIN_ID=101)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
