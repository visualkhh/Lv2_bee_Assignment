cmake_minimum_required(VERSION 3.16)
project(ros_system)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# 스캔 자세일 때만 gripper_cam depth를 RTAB-Map에 넘기는 노드 (scripts/scan_cam_gate.py — 설명은 파일 머리말)
install(PROGRAMS scripts/scan_cam_gate.py DESTINATION lib/${PROJECT_NAME} RENAME scan_cam_gate)
install(PROGRAMS scripts/cuvslam_slam.py DESTINATION lib/${PROJECT_NAME} RENAME cuvslam_slam)
# nvblox 격자 → /map(latched) + 저장 지도 불러오기 (slam:=cuvslam_nvblox — scripts/nvblox_map.py 머리말)
install(PROGRAMS scripts/nvblox_map.py DESTINATION lib/${PROJECT_NAME} RENAME nvblox_map)

# 로봇 모델은 워크스페이스 루트의 urdf/ 에 둔다 (Isaac Sim 등 ROS 밖에서도 같은 파일을 쓰므로).
# 모델 하나 = 폴더 하나: urdf/<이름>/urdf.xacro (+ meshes/) — launch 인자 urdf:=<이름>으로 고름 (기본 default)
# URDF = lv2_module6/urdf. 컨테이너는 /ws/urdf로 마운트(= src/../../urdf), 호스트 빌드는 lv2_module6/urdf(= src/../../../urdf)
# CACHE 아님 — 폴더를 바꾸면 다음 빌드에 바로 반영되게
set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../urdf")
if(NOT IS_DIRECTORY "${URDF_DIR}")
  set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../urdf")
endif()
file(GLOB URDF_XACROS CONFIGURE_DEPENDS "${URDF_DIR}/*/urdf.xacro")
if(NOT URDF_XACROS)
  message(FATAL_ERROR "URDF 없음: ${URDF_DIR}/<이름>/urdf.xacro (워크스페이스 루트의 urdf/ 확인)")
endif()
install(DIRECTORY "${URDF_DIR}/" DESTINATION share/${PROJECT_NAME}/urdf PATTERN "scripts" EXCLUDE)  # 생성·검수 스크립트는 설치 안 함

# --- 테스트: colcon test --packages-select ros_system
if(BUILD_TESTING)
  # URDF: 모델 폴더마다 xacro로 펼쳐지고(mock 인자 포함) 링크·관절 트리가 올바른지
  foreach(X ${URDF_XACROS})
    get_filename_component(D "${X}" DIRECTORY)
    get_filename_component(N "${D}" NAME)
    ament_add_test(test_urdf_${N} GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND bash -c "xacro '${X}' mock_hardware:=true > '${CMAKE_CURRENT_BINARY_DIR}/${N}.urdf' && check_urdf '${CMAKE_CURRENT_BINARY_DIR}/${N}.urdf'"
      TIMEOUT 60)
  endforeach()
  # SLAM 선택(slam:=rtabmap | cuvslam_nvblox)이 맞는 노드로 펼쳐지는지 — GPU 없이 launch 구조만 (test/test_slam_launch.py)
  ament_add_test(test_slam_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/test_slam_launch.py"
    TIMEOUT 60)
endif()

ament_package()
