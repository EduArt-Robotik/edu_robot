find_package(ament_cmake_catch2 REQUIRED)
find_package(Catch2 REQUIRED)

ament_add_catch2(unit_test_kinematic_kalman_filter
  unit_test_kinematic_state_vector.cpp
  unit_test_kinematic_attribute.cpp
  unit_test_kalman_filter_model.cpp
  unit_test_observation_matrix_builder.cpp
)
target_link_libraries(unit_test_kinematic_kalman_filter
  Catch2::Catch2
  eduart-robot
)
ament_target_dependencies(unit_test_kinematic_kalman_filter
  ament_cmake_catch2
)
