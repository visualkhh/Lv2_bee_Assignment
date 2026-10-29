cmake_minimum_required(VERSION 3.16)
project(plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception REQUIRED)
find_package(control REQUIRED)
find_package(ml REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SubGoal.msg
  msg/SubGoalArray.msg
  msg/TaskStatus.msg
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
include_directories(include)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/plan/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/user_input.cpp src/vlm_task_planner.cpp src/coordinator.cpp src/explorer.cpp
  src/mission_solver.cpp src/mission_manager.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS} ${nav_msgs_TARGETS}
  tf2::tf2 tf2_ros::tf2_ros
  ${moveit_msgs_TARGETS} ${control_msgs_TARGETS} ${shape_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${perception_TARGETS} ${control_TARGETS} ${ml_TARGETS} ${msgs_cpp} diagnostic_updater::diagnostic_updater
  ${OpenCV_LIBS})
foreach(node user_input vlm_task_planner coordinator explorer mission_manager)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS user_input vlm_task_planner coordinator explorer mission_manager DESTINATION lib/${PROJECT_NAME})
# skill 시험 도구 (Python) — ros2 run plan skill_test …
install(PROGRAMS scripts/skill_test.py DESTINATION lib/${PROJECT_NAME} RENAME skill_test)

# 파라미터 (bringup이 share/plan/config/plan.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select plan (노드 클래스를 같은 프로세스에 띄워 토픽으로 확인)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinator test/test_coordinator.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=104)
  target_link_libraries(test_coordinator ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_mission_solver test/test_mission_solver.cpp TIMEOUT 60)
  target_link_libraries(test_mission_solver ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_explorer test/test_explorer.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=110)
  target_link_libraries(test_explorer ${PROJECT_NAME}_nodes)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/plan.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
