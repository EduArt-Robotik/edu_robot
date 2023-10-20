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
  eduart-robot-eduard
)
ament_target_dependencies(ethernet-gateway
  "rclcpp"
)
# rosidl_target_interfaces(ethernet-gateway ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Ehternet Gateway Universal Bot
add_executable(ethernet-gateway-universal-bot
  ethernet_gateway_universal_bot_node.cpp
)
target_link_libraries(ethernet-gateway-universal-bot
  eduart-ethernet-gateway
  eduart-robot-universal-bot
  eduart-robot-eduard
)
ament_target_dependencies(ethernet-gateway-universal-bot
  "rclcpp"
)
# rosidl_target_interfaces(ethernet-gateway ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Ehternet Gateway Ohmni Bot
add_executable(ethernet-gateway-ohmni-bot
  ohmni_bot_node.cpp
)
target_link_libraries(ethernet-gateway-ohmni-bot
  eduart-ethernet-gateway
  ohmni-bot
)
ament_target_dependencies(ethernet-gateway-ohmni-bot
  "rclcpp"
)
# rosidl_target_interfaces(ethernet-gateway ${PROJECT_NAME} "rosidl_typesupport_cpp")