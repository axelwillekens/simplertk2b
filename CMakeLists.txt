cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

file(GLOB serialcommfiles
    "lib/serialcomm/*.h"
    "lib/serialcomm/*.c"
)

add_library(serialComm ${serialcommfiles})

add_executable(simplertk2b main.cpp)
target_link_libraries(simplertk2b serialComm)

add_compile_options(-std=gnu99)