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

### Ehternet Gateway Flex Bot
add_executable(ethernet-gateway-flex-bot
  ethernet_gateway_flex_bot_node.cpp
)
target_link_libraries(ethernet-gateway-flex-bot
  eduart-ethernet-gateway
)
ament_target_dependencies(ethernet-gateway-flex-bot
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