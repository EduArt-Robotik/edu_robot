# Library Robot Ohmni Bot
add_library(ohmni-bot
  ohmni_bot.cpp
)
target_include_directories(ohmni-bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ohmni-bot
  eduart-robot
)
