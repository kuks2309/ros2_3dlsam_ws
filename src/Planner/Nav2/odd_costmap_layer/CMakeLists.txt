cmake_minimum_required(VERSION 3.8)
project(odd_costmap_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

set(deps
  rclcpp rclcpp_lifecycle nav_msgs
  nav2_costmap_2d nav2_util pluginlib tf2_ros)

add_library(${PROJECT_NAME} SHARED
  src/odd_corridor_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${deps})

pluginlib_export_plugin_description_file(nav2_costmap_2d odd_costmap_layer_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES odd_costmap_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

# Tests are wired in Phase 3 (Task 5+). BUILD_TESTING block intentionally
# omitted for now; will be re-added when test_odd_corridor_layer.cpp lands.

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${deps})
ament_package()
