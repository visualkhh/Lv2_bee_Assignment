cmake_minimum_required(VERSION 3.16)
project(ml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# ONNX Runtime — 기본은 Isaac ROS vendor(CPU 전용). GPU(CUDA) 빌드를 쓰려면
#   colcon build --cmake-args -DML_ONNXRUNTIME_ROOT=/opt/onnxruntime
#   (/opt/onnxruntime/include/onnxruntime_cxx_api.h · lib/libonnxruntime.so)
set(ML_ONNXRUNTIME_ROOT "" CACHE PATH "CUDA-enabled ONNX Runtime root (include/, lib/)")
find_package(onnxruntime_vendor QUIET)
set(ML_ONNX_FOUND FALSE)
set(ML_ONNX_LINK_TARGET "")
if(ML_ONNXRUNTIME_ROOT)
  find_path(ML_ONNX_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS "${ML_ONNXRUNTIME_ROOT}/include" "${ML_ONNXRUNTIME_ROOT}/include/onnxruntime"
    NO_DEFAULT_PATH)
  find_library(ML_ONNX_LIBRARY onnxruntime
    PATHS "${ML_ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
  if(NOT ML_ONNX_INCLUDE_DIR OR NOT ML_ONNX_LIBRARY)
    message(FATAL_ERROR "ML_ONNXRUNTIME_ROOT=${ML_ONNXRUNTIME_ROOT} — "
      "include/onnxruntime_cxx_api.h 또는 lib/libonnxruntime.so가 없음")
  endif()
  add_library(ml_onnxruntime SHARED IMPORTED)
  set_target_properties(ml_onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ML_ONNX_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ML_ONNX_INCLUDE_DIR}")
  set(ML_ONNX_FOUND TRUE)
  set(ML_ONNX_LINK_TARGET ml_onnxruntime)
  set(ML_ONNX_RPATH "${ML_ONNXRUNTIME_ROOT}/lib")
  message(STATUS "ml: ONNX Runtime GPU root 사용 — ${ML_ONNXRUNTIME_ROOT}")
elseif(onnxruntime_vendor_FOUND)
  set(ML_ONNX_FOUND TRUE)
  set(ML_ONNX_LINK_TARGET onnxruntime::onnxruntime)
endif()
find_package(perception REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/ScoreGrasps.srv
  srv/SetFallback.srv
  msg/SkillContext.msg
  msg/ArmCorrectionDebug.msg
  DEPENDENCIES geometry_msgs std_msgs perception)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)
include_directories(include)

# --- 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리 하나로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/ml/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/grasp_scorer.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${nav2_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${perception_TARGETS}
  ${msgs_cpp} ament_index_cpp::ament_index_cpp diagnostic_updater::diagnostic_updater)
# OnnxPolicy — 모든 학습 정책의 추론 한 곳 (rl_arm_correction · rl_nav_controller · skills · plan이 링크)
#   공유 라이브러리로 설치·공개 (ml::ml_onnx). ONNX Runtime은 PRIVATE — 헤더는 Ort를 앞선언만 함
#   항상 컴파일 (vendor 없으면 스텁 → load()가 실패, 호출 쪽은 바이패스)
add_library(ml_onnx SHARED src/onnx_policy.cpp)
target_compile_features(ml_onnx PUBLIC cxx_std_17)
target_include_directories(ml_onnx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC ml_onnx)
if(ML_ONNX_FOUND)
  target_compile_definitions(ml_onnx PRIVATE ML_HAS_ONNX)
  target_link_libraries(ml_onnx PRIVATE ${ML_ONNX_LINK_TARGET})
  message(STATUS "ml: ONNX Runtime 사용 (${ML_ONNX_LINK_TARGET})")
else()
  message(WARNING "ml: onnxruntime_vendor 없음 — ONNX 없이 빌드 (Δ=0 통과 · 정책 바이패스). "
                  "설치: sudo apt install ros-lyrical-onnxruntime-vendor")
endif()
install(TARGETS ml_onnx EXPORT export_ml_onnx LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_ml_onnx HAS_LIBRARY_TARGET)
# GPU ONNX Runtime을 /opt/onnxruntime 같은 기본 경로 밖에 두면 provider .so를 찾도록 rpath를 박는다.
function(ml_onnx_rpath target)
  if(ML_ONNX_RPATH)
    set_target_properties(${target} PROPERTIES
      BUILD_RPATH "${ML_ONNX_RPATH}"
      INSTALL_RPATH "${ML_ONNX_RPATH}")
  endif()
endfunction()
ml_onnx_rpath(ml_onnx)
foreach(node grasp_scorer)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
  ml_onnx_rpath(${node})
endforeach()
install(TARGETS grasp_scorer DESTINATION lib/${PROJECT_NAME})

# --- rl_nav_controller: Nav2 controller 플러그인 (controller_server가 불러 씀 · ONNX 정책 + inner 바이패스)
add_library(rl_nav_controller SHARED src/rl_nav_controller.cpp)
target_include_directories(rl_nav_controller PUBLIC include)
target_compile_features(rl_nav_controller PUBLIC cxx_std_17)
target_link_libraries(rl_nav_controller PUBLIC rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  nav2_core::nav2_core nav2_util::nav2_util_core nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common pluginlib::pluginlib tf2::tf2 tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS}
  ${diagnostic_msgs_TARGETS})
target_link_libraries(rl_nav_controller PUBLIC ml_onnx ${msgs_cpp})   # ml/srv/SetFallback (런타임 inner 교체)
ml_onnx_rpath(rl_nav_controller)
pluginlib_export_plugin_description_file(nav2_core rl_nav_controller.xml)
install(TARGETS rl_nav_controller LIBRARY DESTINATION lib)

# --- rl_arm_correction: ROS 노드가 아니라 controller_manager가 불러 쓰는 chainable controller 플러그인
add_library(rl_arm_correction SHARED src/rl_arm_correction.cpp)
target_compile_features(rl_arm_correction PUBLIC cxx_std_17)
target_include_directories(rl_arm_correction PUBLIC include)
target_link_libraries(rl_arm_correction controller_interface::controller_interface
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools ament_index_cpp::ament_index_cpp ${perception_TARGETS} ${msgs_cpp} ${std_srvs_TARGETS} ml_onnx)
ml_onnx_rpath(rl_arm_correction)
pluginlib_export_plugin_description_file(controller_interface rl_arm_correction.xml)
install(TARGETS rl_arm_correction LIBRARY DESTINATION lib)

# 파라미터 (bringup이 share/ml/config/ml.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 학습 모델 — lv2_module6/models/<노드>/* → share/ml/model/<노드>/ (기본 model_path=rl_nav_controller/policy.onnx)
#   노드별 폴더로 겹치지 않게 (rl_nav_controller · grasp_scorer …)
set(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models")
if(IS_DIRECTORY "${MODEL_DIR}")
  # yolo는 perception이 share/perception/model에 설치 — ml에서 중복 설치 안 함
  install(DIRECTORY "${MODEL_DIR}/" DESTINATION share/${PROJECT_NAME}/model
    PATTERN ".gitkeep" EXCLUDE PATTERN "yolo" EXCLUDE)
endif()
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- 테스트: colcon test --packages-select ml (노드 클래스를 같은 프로세스에 띄워 토픽으로 확인)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mlp_policy test/test_mlp_policy.cpp TIMEOUT 60)
  target_include_directories(test_mlp_policy PRIVATE include)
  target_compile_definitions(test_mlp_policy PRIVATE MODEL_DIR="${MODEL_DIR}")
  ament_add_gtest(test_onnx_policy test/test_onnx_policy.cpp TIMEOUT 120)
  target_compile_definitions(test_onnx_policy PRIVATE MODEL_DIR="${MODEL_DIR}")
  target_link_libraries(test_onnx_policy ml_onnx)
  ament_add_gtest(test_nav_policy_onnx test/test_nav_policy_onnx.cpp TIMEOUT 120)
  target_include_directories(test_nav_policy_onnx PRIVATE include)
  target_compile_definitions(test_nav_policy_onnx PRIVATE MODEL_DIR="${MODEL_DIR}")
  target_link_libraries(test_nav_policy_onnx ml_onnx)
  ament_add_gtest(test_arm_obs test/test_arm_obs.cpp TIMEOUT 60)
  target_include_directories(test_arm_obs PRIVATE include)
  ament_add_gtest(test_grasp_scorer test/test_grasp_scorer.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=111)
  target_link_libraries(test_grasp_scorer ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_rl_arm_correction test/test_rl_arm_correction.cpp)
  target_link_libraries(test_rl_arm_correction rl_arm_correction)
  ament_add_gtest(test_nav_obs test/test_nav_obs.cpp TIMEOUT 60)
  ament_add_gtest(test_fallback_controller test/test_fallback_controller.cpp TIMEOUT 60)
  target_include_directories(test_fallback_controller PRIVATE include)
  target_include_directories(test_nav_obs PRIVATE include)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/ml.launch.py" --show-args
    TIMEOUT 60)
endif()

# 헤더 (policy.hpp · onnx_policy.hpp · mlp_policy.hpp · arm_obs.hpp · nav_obs.hpp — skills · plan도 씀)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
