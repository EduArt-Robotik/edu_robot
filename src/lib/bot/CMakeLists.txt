# Library for EduArt's Bots
add_library(eduart-bot
  eduard.cpp
  eduard_v2.cpp
  eduard_v3.cpp
  universal_bot.cpp
  rebel_move.cpp
)
target_include_directories(eduart-bot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-bot
  eduart-robot
)
list(APPEND BUILT_LIBRARIES eduart-bot)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)