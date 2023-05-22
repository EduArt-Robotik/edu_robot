add_library(processing-component
  collison_avoidance.cpp
  odometry_estimator.cpp
)
target_link_libraries(processing-component
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
target_compile_features(processing-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(processing-component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  processing-component
  "rclcpp"
  "nav_msgs"
  "tf2_geometry_msgs"
)
