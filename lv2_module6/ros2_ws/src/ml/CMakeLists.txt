cmake_minimum_required(VERSION 3.16)
project(ml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(perception REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/ScoreGrasps.srv
  DEPENDENCIES geometry_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
include_directories(include)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/ml/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/grasp_scorer.cpp src/rl_nav_tuner.cpp src/rl_nav_correction.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${perception_TARGETS} ${msgs_cpp} diagnostic_updater::diagnostic_updater)
foreach(node grasp_scorer rl_nav_tuner rl_nav_correction)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS grasp_scorer rl_nav_tuner rl_nav_correction DESTINATION lib/${PROJECT_NAME})

# --- rl_arm_correction: ROS 노드가 아니라 controller_manager가 불러 쓰는 chainable controller 플러그인
add_library(rl_arm_correction SHARED src/rl_arm_correction.cpp)
target_compile_features(rl_arm_correction PUBLIC cxx_std_17)
target_link_libraries(rl_arm_correction controller_interface::controller_interface
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface rl_arm_correction.xml)
install(TARGETS rl_arm_correction LIBRARY DESTINATION lib)

# 파라미터 (bringup이 share/ml/config/ml.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select ml (노드 클래스를 같은 프로세스에 띄워 토픽으로 확인)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rl_nav_correction test/test_rl_nav_correction.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=103)
  target_link_libraries(test_rl_nav_correction ${PROJECT_NAME}_nodes)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/ml.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
