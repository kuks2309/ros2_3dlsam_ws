cmake_minimum_required(VERSION 3.8)
project(corridor_aware_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(local_odd_obstacle_detector REQUIRED)

set(deps
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  nav2_core nav2_costmap_2d nav2_util
  nav2_regulated_pure_pursuit_controller pluginlib
  local_odd_obstacle_detector)

# Task 1 scaffold: only corridor_aware_controller.cpp exists. Subsequent
# tasks will add state_machine.cpp, lightweight_pursuit.cpp,
# safe_stop_ramp.cpp and the matching gtest binaries.
add_library(${PROJECT_NAME} SHARED
  src/corridor_aware_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${deps})

pluginlib_export_plugin_description_file(nav2_core corridor_aware_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES corridor_aware_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${deps})
ament_package()
