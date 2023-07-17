add_library(eduart-action
  action_manager.cpp
)
target_compile_features(eduart-action PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  eduart-action
  "rclcpp"
)
