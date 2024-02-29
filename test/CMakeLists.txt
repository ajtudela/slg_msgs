ament_add_gtest(test_point2d
  test_point2d.cpp
)
ament_target_dependencies(test_point2d
  geometry_msgs
)

ament_add_gtest(test_polygon
test_polygon.cpp
)
ament_target_dependencies(test_polygon
  geometry_msgs
)

ament_add_gtest(test_segment2d
test_segment2d.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_target_dependencies(test_segment2d
  geometry_msgs
)
target_link_libraries(test_segment2d "${cpp_typesupport_target}")