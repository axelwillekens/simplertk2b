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
find_package(Boost REQUIRED system)

#######################
## Compile with ASAN ##
#######################

# Enable the address sanitizer on Mac and Linux.
# Turn this off by setting the `NO_ASAN` environment variable to `1`
# Cache the environment variable
if (NOT "$ENV{NO_ASAN}" STREQUAL "")
    if ("$ENV{NO_ASAN}" STREQUAL "1")
        set(NO_ASAN "$ENV{NO_ASAN}" CACHE INTERNAL "Copied from environment variable")
    elseif ("$ENV{NO_ASAN}" STREQUAL "0")
        unset (NO_ASAN CACHE)
    endif()
endif()
if(NOT WIN32 AND NOT NO_ASAN)
    if(EXISTS "/etc/arch-release")
        message(STATUS "Note: Compiling with dynamic asan address sanitization and leak checking (\"Arch, as a rule, doesn't include static libs\")")
        set(CMAKE_C_FLAGS      "${CMAKE_C_FLAGS}      -fsanitize=address -fno-omit-frame-pointer")
        set(CMAKE_CXX_FLAGS    "${CMAKE_CXX_FLAGS}    -fsanitize=address -fno-omit-frame-pointer")
        set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    else()
        message(STATUS "Note: Compiling with asan address sanitization and leak checking")
        set(CMAKE_C_FLAGS      "${CMAKE_C_FLAGS}      -fsanitize=address -static-libasan -fno-omit-frame-pointer")
        set(CMAKE_CXX_FLAGS    "${CMAKE_CXX_FLAGS}    -fsanitize=address -static-libasan -fno-omit-frame-pointer")
        set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address -static-libasan -fno-omit-frame-pointer")
    endif()
else()
    message("ASAN disabled: To enable, run: NO_ASAN=0 cmake .")
endif()

#######################
## Build simplertk2b ##
#######################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB serialcommfiles
    "src/serialcomm/*.h"
    "src/serialcomm/*.c"
)

file(GLOB ntripfiles
    "src/ntrip/*.h"
    "src/ntrip/*.c"
)

file(GLOB nmeafiles
    "src/nmealine/*.h"
    "src/nmealine/*.cpp"
)

file(GLOB utmfiles
    "src/utm/*.h"
    "src/utm/*.cpp"
)

file(GLOB simplertk2bfiles
    "src/simplertk2b/*.h"
    "src/simplertk2b/*.cpp"
    "src/simplertk2b_node.cpp"
)

add_executable(simplertk2b_node 
    ${serialcommfiles}
    ${ntripfiles}
    ${nmeafiles}
    ${utmfiles}
    ${simplertk2bfiles}
)
target_link_libraries(simplertk2b_node -lboost_system -pthread)
# message("PATH TO LIBGPS: ${PROJECT_SOURCE_DIR}/lib/libgps.so")
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
