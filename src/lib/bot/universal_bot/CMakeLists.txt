# Library Robot Universal Bot
add_library(eduart-robot-universal-bot
  universal_bot.cpp
)
target_include_directories(eduart-robot-universal-bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-robot-universal-bot
  eduart-robot
)
