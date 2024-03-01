cmake_minimum_required(VERSION 3.5)
project(slg_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################
set(msg_files
  "msg/Segment.msg"
  "msg/SegmentArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_includes INTERFACE)
target_include_directories(${PROJECT_NAME}_includes INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
)

# ############
# # Install ##
# ############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"

  # PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}_includes
  EXPORT export_${PROJECT_NAME}
)

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration ##
# ##################################
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()