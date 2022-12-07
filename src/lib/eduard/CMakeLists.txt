# Library Robot Eduard
add_library(eduart-robot-eduard
  eduard.cpp
)
target_include_directories(eduart-robot-eduard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-robot-eduard
  eduart-robot
)
