cmake_minimum_required(VERSION 3.8)
project(waypoint_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Segment.msg"
  "msg/MissionStatus.msg"
  "action/WaypointMission.action"
  "srv/PauseMission.srv"
  "srv/SkipWaypoint.srv"
)

ament_package()
