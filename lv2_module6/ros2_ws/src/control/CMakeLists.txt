cmake_minimum_required(VERSION 3.16)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ArmSafetyEvent.msg
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
include_directories(include)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/control/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/arm_safety_monitor.cpp src/manual_control.cpp src/body_filter.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action ${sensor_msgs_TARGETS} ${action_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${trajectory_msgs_TARGETS} ${control_msgs_TARGETS} ${msgs_cpp}
  diagnostic_updater::diagnostic_updater urdf::urdf tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
  ament_index_cpp::ament_index_cpp)
foreach(node arm_safety_monitor manual_control body_filter)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS arm_safety_monitor manual_control body_filter DESTINATION lib/${PROJECT_NAME})

# --- BeebotSystemHardware: controller_manager가 URDF <ros2_control>로 불러 쓰는 hardware component 플러그인
add_library(beebot_system_hardware SHARED src/beebot_system_hardware.cpp)
target_compile_features(beebot_system_hardware PUBLIC cxx_std_17)
target_link_libraries(beebot_system_hardware hardware_interface::hardware_interface
  pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle diagnostic_updater::diagnostic_updater)
pluginlib_export_plugin_description_file(hardware_interface control_plugins.xml)
install(TARGETS beebot_system_hardware LIBRARY DESTINATION lib)

# 파라미터 (bringup이 share/control/config/control.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select control (노드 클래스를 같은 프로세스에 띄워 토픽으로 확인)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_safety_monitor test/test_arm_safety_monitor.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=102)
  target_link_libraries(test_arm_safety_monitor ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_manual_control test/test_manual_control.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=105)
  target_link_libraries(test_manual_control ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_body_filter test/test_body_filter.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=108)
  target_link_libraries(test_body_filter ${PROJECT_NAME}_nodes)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
