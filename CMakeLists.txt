cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

file(GLOB serialcommfiles
    "lib/serialcomm/*.h"
    "lib/serialcomm/*.c"
)

file(GLOB ntripfiles
    "lib/ntrip/*.h"
    "lib/ntrip/*.c"
)

add_library(serialComm ${serialcommfiles})
add_library(ntrip ${ntripfiles})

add_executable(simplertk2b main.cpp)
target_link_libraries(simplertk2b serialComm ntrip pthread)

add_compile_options(-std=gnu99)