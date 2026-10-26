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
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs diagnostic_msgs)

# --- C++ 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/perception/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/body_filter.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater urdf::urdf tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
  ament_index_cpp::ament_index_cpp)
foreach(node body_filter)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS body_filter DESTINATION lib/${PROJECT_NAME})

# YOLO 노드 (Python, scripts/perception_node.py) — 메시지 패키지는 이 CMake가 만든다
install(PROGRAMS scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node)

# 파라미터 (bringup이 share/perception/config/perception.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_filter test/test_body_filter.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=108)
  target_link_libraries(test_body_filter ${PROJECT_NAME}_nodes)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/perception.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
