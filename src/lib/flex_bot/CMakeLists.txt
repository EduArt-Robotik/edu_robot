# Library Robot Flex Bot
add_library(eduart-robot-flex-bot
  flex_bot.cpp
)
target_include_directories(eduart-robot-flex-bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-robot-flex-bot
  eduart-robot
)
