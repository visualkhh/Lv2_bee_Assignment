cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# stand_test.launch.py 전용 — Nav2 없이 /cmd_vel_mux → /cmd_vel (수동 주행이 바퀴까지 가게)
install(PROGRAMS scripts/cmd_vel_relay.py DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_relay)

# --- 테스트: colcon test --packages-select bringup
if(BUILD_TESTING)
  # launch 파일이 문법·인자 선언 오류 없이 읽히는지 (노드는 띄우지 않음 — 실행 흐름은 docker/test-faker)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/bringup.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_package()
