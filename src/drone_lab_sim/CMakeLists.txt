cmake_minimum_required(VERSION 3.8)
project(drone_lab_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# -- fake_drone  (simulates MAVROS topics + services for N drones) ----
add_executable(fake_drone src/fake_drone.cpp)
ament_target_dependencies(fake_drone
  rclcpp std_msgs geometry_msgs mavros_msgs std_srvs)

# -- student_cmd (publishes test setpoints for one drone) -------------
add_executable(student_cmd src/student_cmd.cpp)
ament_target_dependencies(student_cmd
  rclcpp std_msgs geometry_msgs mavros_msgs std_srvs)

# -- install ----------------------------------------------------------
install(TARGETS fake_drone student_cmd
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
