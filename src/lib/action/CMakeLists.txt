add_library(eduart-action
  action_manager.cpp
  motor_action.cpp
  indicate_rejected_action.cpp
)
target_link_libraries(eduart-action
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  "${sensor_msgs_TARGETS}"
  "${diagnostic_msgs_TARGETS}"
  diagnostic_updater::diagnostic_updater
)
target_compile_features(eduart-action PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
list(APPEND BUILT_LIBRARIES eduart-action)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)