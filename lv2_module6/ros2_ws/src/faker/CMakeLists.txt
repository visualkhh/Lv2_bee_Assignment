cmake_minimum_required(VERSION 3.16)
project(faker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(perception REQUIRED)
find_package(ml REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosx_introspection REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로, 실행 파일은 src/<노드>_main.cpp (선언·설명은 include/faker/<노드>.hpp)
#   📦 대역: fake_camera(RealSense · 장면은 시뮬레이터) · fake_opencr(OpenCR — 테스트베드의 유일한 OpenCR) · ml 대역 fake_grasp_scorer · fake_rl_nav_tuner · fake_rl_nav_correction (ml 노드와 1:1)
#   move_group은 진짜를 쓴다 (ros_system use_moveit — 테스트베드도 mock 컨트롤러로 동작)
#   호스트 파일 연동: monitor_manager(토픽 → 파일) · input_manager(파일 → 토픽)
set(NODES fake_camera fake_opencr fake_grasp_scorer fake_rl_nav_tuner fake_rl_nav_correction monitor_manager input_manager service_action_monitor service_action_input)
add_library(${PROJECT_NAME}_nodes STATIC
  src/fake_camera.cpp src/fake_opencr.cpp src/fake_grasp_scorer.cpp src/fake_rl_nav_tuner.cpp src/fake_rl_nav_correction.cpp src/monitor_manager.cpp src/input_manager.cpp src/service_action_monitor.cpp src/service_action_input.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS} ${std_msgs_TARGETS} ${control_msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${rcl_interfaces_TARGETS}
  ${moveit_msgs_TARGETS} ${moveit_task_constructor_msgs_TARGETS} ${perception_TARGETS} ${ml_TARGETS}
  diagnostic_updater::diagnostic_updater tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs rosx_introspection::rosx_introspection cv_bridge::cv_bridge ament_index_cpp::ament_index_cpp ${OpenCV_LIBS})
foreach(node ${NODES})
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})

# --- ml/RlArmCorrection 대역 컨트롤러 플러그인 (Δ=0 통과)
add_library(fake_rl_arm_correction SHARED src/fake_rl_arm_correction.cpp)
target_include_directories(fake_rl_arm_correction PUBLIC include)
target_compile_features(fake_rl_arm_correction PUBLIC cxx_std_17)
target_link_libraries(fake_rl_arm_correction controller_interface::controller_interface
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface faker_plugins.xml)
install(TARGETS fake_rl_arm_correction LIBRARY DESTINATION lib)

# maps: 코스 지도 (lv2_module6/course/tools/build_map.py가 만듦) — 진짜 Nav2용 map_server
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select faker
if(BUILD_TESTING)
  # launch 파일이 문법·인자 선언 오류 없이 읽히는지 (노드는 띄우지 않음 — 실행 흐름은 docker/test-faker)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/faker.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_package()
