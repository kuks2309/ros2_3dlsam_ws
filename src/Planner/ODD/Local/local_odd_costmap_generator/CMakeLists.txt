cmake_minimum_required(VERSION 3.8)
project(local_odd_costmap_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# --- Library (ROS-independent algorithm) ---
add_library(corridor_rasterizer_lib STATIC
  src/corridor_rasterizer.cpp
)
target_include_directories(corridor_rasterizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corridor_rasterizer_lib
  nav_msgs
)

# --- Executable ---
add_executable(local_odd_costmap_generator_node
  src/local_odd_costmap_generator_node.cpp
)
target_include_directories(local_odd_costmap_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_odd_costmap_generator_node corridor_rasterizer_lib)
ament_target_dependencies(local_odd_costmap_generator_node
  rclcpp
  nav_msgs
)

# --- Installation ---
install(TARGETS corridor_rasterizer_lib local_odd_costmap_generator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
