# ManualActors — Gazebo GUI 버튼 플러그인
#
# Gazebo GUI에 "사람 소환 / 박스 소환 / 수동 모델 삭제" 버튼을 붙인다.
# 소환은 gz-transport의 /world/<world>/create 서비스로 빈 후보 지점에 바로 생성한다.
# 임의 위치에 직접 놓고 싶으면 Gazebo 기본 Spawn/드래그를 사용한다.
#
# 빌드:
#   cmake -S . -B build
#   cmake --build build -j
#
# start.sh가 build/libManualActors.so를 GZ_GUI_PLUGIN_PATH에 넣고
# gui.config의 <plugin filename="ManualActors">로 로드한다.

cmake_minimum_required(VERSION 3.22)
project(ManualActors VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(gz-gui REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin COMPONENTS register REQUIRED)
find_package(gz-sim REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Quick Widgets)

add_library(ManualActors SHARED
        ManualActors.cc
        ManualActors.hh
        ManualActors.qml
)

target_link_libraries(ManualActors PRIVATE
  gz-gui::gz-gui
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-plugin::gz-plugin-register
  gz-sim::gz-sim
  Qt6::Core
  Qt6::Quick
  Qt6::Widgets
)

# Gazebo GUI는 :/<plugin filename>/<plugin filename>.qml 을 찾는다.
qt_add_resources(ManualActors manual_actors_resources
  PREFIX "/ManualActors"
  FILES ManualActors.qml
)

install(TARGETS ManualActors
  LIBRARY DESTINATION lib)
