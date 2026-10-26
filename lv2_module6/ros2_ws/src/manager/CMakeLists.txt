cmake_minimum_required(VERSION 3.16)
project(manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(control REQUIRED)
# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/manager/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/state_manager.cpp src/restart.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS} ${rtabmap_msgs_TARGETS}
  ${control_TARGETS} diagnostic_updater::diagnostic_updater)
add_executable(state_manager src/state_manager_main.cpp)
target_link_libraries(state_manager ${PROJECT_NAME}_nodes)
install(TARGETS state_manager DESTINATION lib/${PROJECT_NAME})

# 파라미터 (bringup이 share/manager/config/manager.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select manager
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_manager test/test_state_manager.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=109)
  target_link_libraries(test_state_manager ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_restart test/test_restart.cpp TIMEOUT 60)
  target_link_libraries(test_restart ${PROJECT_NAME}_nodes)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/manager.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_package()
