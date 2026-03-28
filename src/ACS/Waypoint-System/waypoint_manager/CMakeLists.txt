cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(waypoint_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(waypoint_manager_node
  src/main.cpp
  src/waypoint_manager_node.cpp
  src/action_chainer.cpp
  src/segment_planner.cpp
  src/retry_policy.cpp)
target_include_directories(waypoint_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_manager_node
  rclcpp rclcpp_action amr_interfaces waypoint_interfaces
  geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
target_compile_features(waypoint_manager_node PRIVATE cxx_std_17)

install(TARGETS waypoint_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
