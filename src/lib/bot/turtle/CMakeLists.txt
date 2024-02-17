# Library Robot Eduard
add_library(eduart-turtle
  turtle.cpp
)
target_include_directories(eduart-turtle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-turtle
  eduart-robot
)
