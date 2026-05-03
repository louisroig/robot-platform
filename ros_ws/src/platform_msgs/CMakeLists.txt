cmake_minimum_required(VERSION 3.8)
project(platform_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/SafetyState.msg"
  "msg/AerialImageMeta.msg"
)

set(srv_files
  "srv/ResetSafety.srv"
  "srv/RetrieveLatestImage.srv"
  "srv/Georeference.srv"
)

set(action_files
  "action/RunMappingMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
