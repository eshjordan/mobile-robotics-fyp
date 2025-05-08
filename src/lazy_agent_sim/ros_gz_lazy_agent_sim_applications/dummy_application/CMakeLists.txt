cmake_minimum_required(VERSION 3.5)

project(dummy_application)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_add_library(robot_driver_component
#   src/main.cpp
# )

# rclcpp_components_register_node(
#     robot_driver_component
#     PLUGIN "lazy_agent_sim::RobotDriver"
#     EXECUTABLE robot_driver
# )

# ament_export_targets(export_vincent_driver_component)
# install(TARGETS vincent_driver_component
#         EXPORT export_vincent_driver_component
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )

ament_auto_package()
