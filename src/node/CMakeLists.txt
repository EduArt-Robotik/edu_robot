## Build Nodes
add_executable(iotbot-shield
  iotbot_shield_node.cpp
)
target_link_libraries(iotbot-shield
  eduart-iotbot-shield
)
ament_target_dependencies(iotbot-shield
  "rclcpp"
)
# rosidl_target_interfaces(iotbot-shield ${PROJECT_NAME} "rosidl_typesupport_cpp")
