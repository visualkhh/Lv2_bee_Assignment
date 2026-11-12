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
# 인지 노드 C++판 (perception_node) — 영상·YOLO·AprilTag·바코드
# AprilTag: OpenCV 4.7+는 objdetect(본체) · 4.6 이하는 contrib aruco — 있는 쪽을 씀 (젯슨 4.8 · PC 도커 4.6)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs OPTIONAL_COMPONENTS objdetect aruco)  # imgcodecs = 테스트 사진 읽기
find_package(message_filters REQUIRED)
# ONNX Runtime: 기본은 ROS 패키지 onnxruntime_vendor (CPU) · GPU(CUDA)판이 따로 깔려 있으면 ONNXRUNTIME_ROOT로 그 폴더를 줌
#   (예: colcon build --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime-gpu — include/onnxruntime_cxx_api.h · lib/libonnxruntime.so)
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "GPU ONNX Runtime 폴더 (비면 /opt/onnxruntime → onnxruntime_vendor)")
if(NOT ONNXRUNTIME_ROOT AND EXISTS "/opt/onnxruntime/lib/libonnxruntime.so")
  set(ONNXRUNTIME_ROOT "/opt/onnxruntime")  # 젯슨: 팀이 CUDA판 ONNX Runtime을 여기 깔아 둠 (yolo_detector_cpp와 같은 자리)
endif()
if(ONNXRUNTIME_ROOT)
  find_path(ORT_INC onnxruntime_cxx_api.h PATHS "${ONNXRUNTIME_ROOT}/include" "${ONNXRUNTIME_ROOT}/include/onnxruntime" NO_DEFAULT_PATH)
  find_library(ORT_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES IMPORTED_LOCATION "${ORT_LIB}" INTERFACE_INCLUDE_DIRECTORIES "${ORT_INC}"
    INTERFACE_LINK_OPTIONS "LINKER:-rpath,${ONNXRUNTIME_ROOT}/lib")
  message(STATUS "ONNX Runtime: ${ORT_LIB}")
else()
  find_package(onnxruntime_vendor REQUIRED)
endif()
# 바코드 (zxing-cpp C++, apt libzxing-dev) — 없으면 바코드 없이 빌드 (진단에 '없음')
find_package(ZXing QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  msg/MissionSignal.msg
  msg/HandTarget.msg
  msg/HandRelative.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs diagnostic_msgs)
rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)  # 같은 패키지 메시지를 C++ 노드가 씀

# --- C++ 노드: 클래스 구현(src/<노드>.cpp)은 라이브러리로 묶고, 실행 파일(src/<노드>_main.cpp)과 테스트가 함께 링크
#     선언·설명은 include/perception/<노드>.hpp
add_library(${PROJECT_NAME}_nodes STATIC src/body_filter.cpp src/signal_decoder.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater urdf::urdf tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
  ament_index_cpp::ament_index_cpp ${msgs_cpp})
foreach(node body_filter signal_decoder)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_nodes)
endforeach()
install(TARGETS body_filter signal_decoder DESTINATION lib/${PROJECT_NAME})

# YOLO 인지 노드 — C++판이 perception_node (bringup·launch가 이 이름으로 띄움, 11/11 젯슨 CPU 때문에 C++로)
#   계산 함수 perception_core · YOLO 실행 yolo_onnx · 노드 perception_node (설명은 include/perception/<이름>.hpp)
add_library(${PROJECT_NAME}_cpp STATIC src/perception_core.cpp src/yolo_onnx.cpp src/perception_node.cpp)
target_include_directories(${PROJECT_NAME}_cpp PUBLIC include)
target_compile_features(${PROJECT_NAME}_cpp PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros ament_index_cpp::ament_index_cpp
  message_filters::message_filters ${OpenCV_LIBS} onnxruntime::onnxruntime ${msgs_cpp})
if(ZXing_FOUND)
  target_link_libraries(${PROJECT_NAME}_cpp PUBLIC ZXing::ZXing)
  message(STATUS "바코드: zxing-cpp ${ZXing_VERSION}")
else()
  message(WARNING "zxing-cpp 없음 (apt install libzxing-dev) — perception_node가 바코드를 못 읽음")
endif()
add_executable(perception_node src/perception_node_main.cpp)
target_link_libraries(perception_node ${PROJECT_NAME}_cpp)
if(ONNXRUNTIME_ROOT)
  # GPU판 ONNX Runtime이 실행 때도 먼저 잡히게 — ROS setup이 LD_LIBRARY_PATH에 넣는 CPU판(onnxruntime_vendor)보다
  # 실행 파일에 적힌 경로(RPATH)가 앞서도록 (기본 RUNPATH는 LD_LIBRARY_PATH에 밀림)
  target_link_options(perception_node PRIVATE "LINKER:--disable-new-dtags")
endif()
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
# 예전 파이썬판 (scripts/perception_node.py) — 되돌릴 때: launch에서 executable을 perception_node_py로
install(PROGRAMS scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node_py)
# 손 카메라 상대값 (Python, scripts/hand_relative.py + 계산 모듈 hand_geom.py를 같은 폴더에)
install(PROGRAMS scripts/hand_relative.py DESTINATION lib/${PROJECT_NAME} RENAME hand_relative)
install(FILES scripts/hand_geom.py DESTINATION lib/${PROJECT_NAME})

# 파라미터 (bringup이 share/perception/config/perception.yaml을 노드에 넘긴다)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 패키지 단독 실행용 (전체 로봇은 bringup)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 학습 가중치 — 워크스페이스 공용 lv2_module6/models/yolo/*.pt · *.onnx 를 share에 설치
# (urdf와 같은 패턴: ROS 밖 train_yolo.py가 쓰는 파일을 패키지가 가져옴)
set(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models/yolo" CACHE PATH "lv2_module6/models/yolo")
file(GLOB MODEL_FILES "${MODEL_DIR}/*.pt" "${MODEL_DIR}/*.onnx")  # .onnx = 젯슨용 (ultralytics가 onnxruntime으로 읽음)
if(MODEL_FILES)
  install(FILES ${MODEL_FILES} DESTINATION share/${PROJECT_NAME}/model)
else()
  message(WARNING "YOLO 가중치 없음 (${MODEL_DIR}/*.pt · *.onnx) — 노드는 빈 목록 발행")
endif()

# --- 테스트: colcon test --packages-select perception
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node test/test_node.py TIMEOUT 120 ENV ROS_DOMAIN_ID=101)
  ament_add_pytest_test(test_hand_relative test/test_hand_relative.py TIMEOUT 60)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_filter test/test_body_filter.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=108)
  target_link_libraries(test_body_filter ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_signal test/test_signal.cpp TIMEOUT 60)
  target_link_libraries(test_signal ${PROJECT_NAME}_nodes)
  ament_add_gtest(test_perception_core test/test_perception_core.cpp TIMEOUT 60
    ENV PERCEPTION_TEST_DATA=${CMAKE_CURRENT_SOURCE_DIR}/test/data)
  target_link_libraries(test_perception_core ${PROJECT_NAME}_cpp)
  ament_add_test(test_launch GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ros2 launch "${CMAKE_CURRENT_SOURCE_DIR}/launch/perception.launch.py" --show-args
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
