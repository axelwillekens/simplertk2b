cmake_minimum_required(VERSION 3.5)
project(simplertk2b_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

#######################
## Build simplertk2b ##
#######################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(utm src/UTM.cpp)
add_executable(simplertk2b_node src/simplertk2b_node.cpp src/simplertk2bsystem.cpp)
target_link_libraries(simplertk2b_node
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libgps.so
  utm
)
message("PATH TO LIBGPS: ${PROJECT_SOURCE_DIR}/lib/libgps.so")
ament_target_dependencies(simplertk2b_node rclcpp geometry_msgs)

install(TARGETS
  simplertk2b_node
  DESTINATION lib/${PROJECT_NAME}
)

########################
## launch simplertk2b ##
########################

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
