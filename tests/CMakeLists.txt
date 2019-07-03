add_executable(
  unit_tests
  TestQuadtree.cpp
)

target_link_libraries(
  unit_tests
  PUBLIC
  spatial_data_structure
  PRIVATE
  gtest_main
)

add_test(
  NAME
  unit_tests
  COMMAND
  $<TARGET_FILE:unit_tests>
)
