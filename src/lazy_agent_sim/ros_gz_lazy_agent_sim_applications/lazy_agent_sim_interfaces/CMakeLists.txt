cmake_minimum_required(VERSION 3.8)
project(lazy_agent_sim_interfaces)

## Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Centroid.msg"
  "msg/Boundary.msg"
  "msg/EpuckKnowledgeRecord.msg"
  "msg/EpuckKnowledgePacket.msg"
)


ament_auto_package()
