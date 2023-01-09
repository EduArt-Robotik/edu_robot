find_package(ament_cmake_catch2 REQUIRED)
find_package(Catch2 REQUIRED)

ament_add_catch2(unit_test_kinematic_state_vector
  unit_test_kinematic_state_vector.cpp
  unit_test_kinematic_attribute.cpp
)
target_link_libraries(unit_test_kinematic_state_vector
  Catch2::Catch2
  eduart-robot
)
ament_target_dependencies(unit_test_kinematic_state_vector
  ament_cmake_catch2
)
