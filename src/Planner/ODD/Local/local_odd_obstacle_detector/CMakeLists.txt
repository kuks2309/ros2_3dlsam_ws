cmake_minimum_required(VERSION 3.8)
project(local_odd_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Message generation ──
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CorridorObstacleStatus.msg"
  DEPENDENCIES std_msgs
)

# ── ObstacleDetector Library (testable without ROS spin) ──
add_library(obstacle_detector_lib STATIC
  src/obstacle_detector.cpp
)
target_include_directories(obstacle_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obstacle_detector_lib
  nav_msgs
  sensor_msgs
)

# ── Main Node Executable ──
add_executable(local_odd_obstacle_detector_node
  src/main.cpp
  src/obstacle_detector_node.cpp
)
target_include_directories(local_odd_obstacle_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_odd_obstacle_detector_node
  obstacle_detector_lib
)
ament_target_dependencies(local_odd_obstacle_detector_node
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(local_odd_obstacle_detector_node ${cpp_typesupport_target})

# ── Install ──
install(TARGETS obstacle_detector_lib local_odd_obstacle_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# ── Tests ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_detector test/test_obstacle_detector.cpp)
  target_link_libraries(test_obstacle_detector obstacle_detector_lib)
  target_include_directories(test_obstacle_detector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_obstacle_detector
    nav_msgs
    sensor_msgs
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
