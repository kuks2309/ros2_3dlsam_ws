cmake_minimum_required(VERSION 3.8)
project(local_odd_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(route_graph_builder REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Message generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalOddSegment.msg"
  "msg/LocalOddMap.msg"
  DEPENDENCIES std_msgs
)

# --- Library (ROS-independent algorithm) ---
add_library(local_odd_core_lib
  src/local_odd_core.cpp
)
target_include_directories(local_odd_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(local_odd_core_lib
  nav_msgs
)

# --- Executable ---
add_executable(local_odd_generator_node
  src/local_odd_generator_node.cpp
)
target_include_directories(local_odd_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_odd_generator_node local_odd_core_lib)
ament_target_dependencies(local_odd_generator_node
  rclcpp
  nav_msgs
  std_msgs
  route_graph_builder
)

# Link generated message typesupport to node only
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(local_odd_generator_node ${cpp_typesupport_target})

# --- Installation ---
install(TARGETS local_odd_core_lib local_odd_generator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz2/ DESTINATION share/${PROJECT_NAME}/rviz2)
install(PROGRAMS
  scripts/publish_loop_path.py
  scripts/auto_test_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
