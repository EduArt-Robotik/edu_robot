add_library(eduart-event
  event_manager.cpp
)
target_compile_features(eduart-event PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-event PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  eduart-event
  "rclcpp"
)
