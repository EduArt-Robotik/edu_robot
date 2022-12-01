## Build Nodes
### IotBot
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

### Ehternet Gateway
add_executable(ethernet-gateway
  ethernet_gateway_bot_node.cpp
)
target_link_libraries(ethernet-gateway
  eduart-ethernet-gateway
)
ament_target_dependencies(ethernet-gateway
  "rclcpp"
)
# rosidl_target_interfaces(ethernet-gateway ${PROJECT_NAME} "rosidl_typesupport_cpp")