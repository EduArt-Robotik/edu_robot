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

### Can Gateway Universal Bot
add_executable(can-gateway-universal-bot
  can_gateway_universal_bot_node.cpp
)
target_link_libraries(can-gateway-universal-bot
  eduart-robot-universal-bot
  eduart-hardware-can
)
ament_target_dependencies(can-gateway-universal-bot
  "rclcpp"
)

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

### Igus Can Turtle Bot
add_executable(igus-can-turtle-bot
  igus_can_turtle_bot_node.cpp
)
target_link_libraries(igus-can-turtle-bot
  igus
  eduart-turtle
  eduart-hardware-can
)
ament_target_dependencies(igus-can-turtle-bot
  "rclcpp"
)
