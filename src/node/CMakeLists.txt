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

### Eduard Ehternet Gateway
add_executable(eduard-ethernet-gateway
  eduard_ethernet_gateway_bot_node.cpp
)
target_link_libraries(eduard-ethernet-gateway
  eduart-ethernet-gateway
  eduart-bot
)
ament_target_dependencies(eduard-ethernet-gateway
  "rclcpp"
)

### Universal Ehternet Gateway Bot
add_executable(universal-ethernet-gateway-bot
  universal_ethernet_gateway_bot_node.cpp
)
target_link_libraries(universal-ethernet-gateway-bot
  eduart-ethernet-gateway
  eduart-bot
)
ament_target_dependencies(universal-ethernet-gateway-bot
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
