cmake_minimum_required(VERSION 3.8)
project(route_graph_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Message generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RouteNode.msg"
  "msg/RouteEdge.msg"
  "msg/RouteGraph.msg"
  DEPENDENCIES std_msgs
)

# --- Library ---
add_library(route_graph_lib
  src/route_graph.cpp
  src/graph_visualizer.cpp
)
target_include_directories(route_graph_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(route_graph_lib
  rclcpp
  geometry_msgs
  visualization_msgs
)

# --- Executable ---
add_executable(route_graph_builder_node
  src/route_graph_builder_node.cpp
)
target_include_directories(route_graph_builder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_graph_builder_node route_graph_lib)
ament_target_dependencies(route_graph_builder_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Link generated messages to targets
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(route_graph_builder_node ${cpp_typesupport_target})
target_link_libraries(route_graph_lib ${cpp_typesupport_target})

# --- Installation ---
install(TARGETS route_graph_lib route_graph_builder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz2/ DESTINATION share/${PROJECT_NAME}/rviz2)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
