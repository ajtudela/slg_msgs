cmake_minimum_required(VERSION 3.8)
project(simple_laser_geometry)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)

## Find system libraries
find_package(PCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
set(msg_files
  "msg/Segment.msg"
  "msg/SegmentStamped.msg"
  "msg/SegmentArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## ament specific configuration ##
###################################
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(slg_conversions src/conversions.cpp)
ament_target_dependencies(slg_conversions std_msgs geometry_msgs tf2 pcl_ros PCL)
target_link_libraries(slg_conversions ${PCL_LIBRARIES})
rosidl_target_interfaces(slg_conversions ${PROJECT_NAME} "rosidl_typesupport_cpp")

#############
## Install ##
#############
install(TARGETS slg_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()