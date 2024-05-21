## Build Nodes
### IotBot
add_executable(iotbot-shield
  iotbot_shield_node.cpp
)
target_link_libraries(iotbot-shield
  eduart-iotbot-shield
  eduart-bot
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
  eduart-bot
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
  eduart-bot
)
ament_target_dependencies(ethernet-gateway-universal-bot
  "rclcpp"
)

### Eduard 360 Pi Bot
add_executable(eduard-360-pi-bot
  eduard_360_pi_node.cpp
)
target_link_libraries(eduard-360-pi-bot
  eduart-bot
  eduart-hardware-can
)
ament_target_dependencies(eduard-360-pi-bot
  "rclcpp"
)

### Universal Bot 360
add_executable(universal-360-pi-bot
  universal_360_pi_bot_node.cpp
)
target_link_libraries(universal-360-pi-bot
  eduart-bot
  eduart-hardware-can
)
ament_target_dependencies(universal-360-pi-bot
  "rclcpp"
)

### Rebel Move Bot
add_executable(rebel-move-360-pi-bot
  rebel_move_360_pi_bot_node.cpp
)
target_link_libraries(rebel-move-360-pi-bot
  igus
  eduart-bot
  eduart-hardware-can
)
ament_target_dependencies(rebel-move-360-pi-bot
  "rclcpp"
)
