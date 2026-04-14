cmake_minimum_required(VERSION 3.14)
project(drone_lab_connector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ── ROS 2 dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Qt (prefer Qt5, accept Qt6) ─────────────────────────────────────────────
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# ── Other dependencies ───────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Headers ──────────────────────────────────────────────────────────────────
include_directories(include)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(drone_lab_connector
  src/main.cpp
  src/config_manager.cpp
  src/safety_node.cpp
  src/main_window.cpp
  include/drone_lab_connector/main_window.hpp
)

target_link_libraries(drone_lab_connector
  Qt${QT_VERSION_MAJOR}::Widgets
  Eigen3::Eigen
  yaml-cpp
)

ament_target_dependencies(drone_lab_connector
  rclcpp
  std_msgs
  geometry_msgs
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS drone_lab_connector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
