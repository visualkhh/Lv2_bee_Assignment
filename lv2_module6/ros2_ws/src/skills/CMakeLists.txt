cmake_minimum_required(VERSION 3.16)
project(skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(perception REQUIRED)
find_package(control REQUIRED)
find_package(ml REQUIRED)
find_package(plan REQUIRED)

# --- skill 노드: 컴포넌트 하나 = skill 하나 (src/<skill>_skill.cpp) · 공통은 skill_node(액션 틀) · arm_io(팔) · stand_plan(세우기 경유점) · look_plan(inspect 카메라 자세)
add_library(${PROJECT_NAME} SHARED
  src/skill_node.cpp src/arm_io.cpp src/stand_plan.cpp src/look_plan.cpp src/far_plan.cpp
  src/navigate_skill.cpp src/approach_skill.cpp src/pose_skill.cpp src/inspect_skill.cpp
  src/pick_skill.cpp src/place_skill.cpp src/load_skill.cpp src/unload_skill.cpp src/stand_skill.cpp
  src/pick_far_skill.cpp)  # pick_far: 박스 먼 쪽 퍽 (far_plan 경유점)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${ml_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action rclcpp_components::component
  diagnostic_updater::diagnostic_updater ament_index_cpp::ament_index_cpp tf2::tf2 tf2_ros::tf2_ros
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS} ${moveit_msgs_TARGETS} ${control_msgs_TARGETS} ${shape_msgs_TARGETS} ${trajectory_msgs_TARGETS}
  ${perception_TARGETS} ${control_TARGETS} ${ml_TARGETS} ml::ml_onnx ${plan_TARGETS})
foreach(cls NavigateSkill ApproachSkill PoseSkill InspectSkill PickSkill PlaceSkill LoadSkill UnloadSkill StandSkill PickFarSkill)
  rclcpp_components_register_nodes(${PROJECT_NAME} "skills::${cls}")
endforeach()
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch params DESTINATION share/${PROJECT_NAME})

# 학습 정책 — 워크스페이스 공용 lv2_module6/models/skills/<skill>/ (meta.yaml · <skill>.onnx · <skill>.policy) → share/skills/model
set(SKILLS_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models/skills" CACHE PATH "lv2_module6/models/skills")
if(EXISTS "${SKILLS_MODEL_DIR}")
  install(DIRECTORY "${SKILLS_MODEL_DIR}/" DESTINATION share/${PROJECT_NAME}/model
    FILES_MATCHING PATTERN "meta.yaml" PATTERN "*.onnx" PATTERN "*.policy")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stand_plan test/test_stand_plan.cpp TIMEOUT 60)
  target_link_libraries(test_stand_plan ${PROJECT_NAME})
  ament_add_gtest(test_far_plan test/test_far_plan.cpp TIMEOUT 60)  # 먼 퍽 경유점 = 시뮬 pick_env 먼 퍽 표
  target_link_libraries(test_far_plan ${PROJECT_NAME})
  ament_add_gtest(test_look_plan test/test_look_plan.cpp TIMEOUT 60)
  target_link_libraries(test_look_plan ${PROJECT_NAME})
  ament_add_gtest(test_skills test/test_skills.cpp TIMEOUT 60)
  target_link_libraries(test_skills ${PROJECT_NAME})
  target_compile_definitions(test_skills PRIVATE MODEL_DIR="${SKILLS_MODEL_DIR}")
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/skills.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action plan ml)
ament_package()
