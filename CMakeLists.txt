cmake_minimum_required(VERSION 3.5)
project(ipa_building_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MapSegmentation.action"
  "action/FindRoomSequenceWithCheckpoints.action"
  "action/RoomExploration.action"
  "msg/RoomInformation.msg"
  "msg/RoomSequence.msg"
  "srv/CheckCoverage.srv"
  "srv/ExtractAreaMapFromLabeledMap.srv"
  DEPENDENCIES geometry_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
