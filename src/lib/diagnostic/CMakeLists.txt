# Library Robot Eduard
add_library(eduart-diagnostic
  diagnostic_component.cpp
)
target_include_directories(eduart-diagnostic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduart-diagnostic

)
ament_target_dependencies(
  eduart-diagnostic
  "diagnostic_updater"
  "diagnostic_msgs"
)
list(APPEND BUILT_LIBRARIES eduart-diagnostic)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)