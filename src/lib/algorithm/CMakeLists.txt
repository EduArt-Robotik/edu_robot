add_library(eduart-algorithm
  rotation.cpp
  geometry.cpp
  pid_controller.cpp
)
target_link_libraries(eduart-algorithm
  "${cpp_typesupport_target}"
  Eigen3::Eigen
  "${geometry_msgs_TARGETS}"
  tf2::tf2
)
target_compile_features(eduart-algorithm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
list(APPEND BUILT_LIBRARIES eduart-algorithm)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
