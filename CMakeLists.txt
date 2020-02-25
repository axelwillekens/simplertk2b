cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system regex thread)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

add_executable(simplertk2b main.cpp src/simplertk2b.cpp)
target_link_libraries(simplertk2b
  boost_system
  boost_regex
  boost_thread
)