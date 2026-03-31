cmake_minimum_required(VERSION 3.8)
project(drone_lab_fencer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# -- safety_fencer ---------------------------------------------------
add_executable(safety_fencer src/safety_fencer.cpp)
ament_target_dependencies(safety_fencer
  rclcpp std_msgs geometry_msgs mavros_msgs std_srvs)

# -- takeoff_service -------------------------------------------------
add_executable(takeoff_service src/takeoff_service.cpp)
ament_target_dependencies(takeoff_service
  rclcpp std_msgs geometry_msgs mavros_msgs std_srvs)

# -- install ---------------------------------------------------------
install(TARGETS safety_fencer takeoff_service
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
