cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (NOT CMAKE_COMPILER_IS_GNUCC)
    message(FATAL_ERROR "You need to build with GCC")
endif()

# according to https://gcc.gnu.org/projects/cxx-status.html we need at least 8 for full C++17 support
if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
   message(FATAL_ERROR "You need at least gcc 8  (found gcc version ${CMAKE_CXX_COMPILER_VERSION})")
endif()

# Note: consider -Werror instead of -Wall to make all warnings fatal
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g -std=gnu99 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment -Wno-unknown-pragmas -fms-extensions -fdiagnostics-color -Og")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++17 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment -Wno-unknown-pragmas -fms-extensions -fdiagnostics-color -pthread -Og")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")

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

# Compilation project

file(GLOB gpsfiles
    "gps/serialcomm/*.h"
    "gps/serialcomm/*.c"
    "gps/ntrip/*.h"
    "gps/ntrip/*.c"
    "gps/simplertk2b/*.h"
    "gps/simplertk2b/*.cpp"
)

add_library(gps ${gpsfiles} )

# SET (BOOST_LIBRARYDIR "/usr/local/lib")

find_package(Boost 1.72 REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})  

    add_executable(simplertk2b main.cpp)
    target_link_libraries(simplertk2b gps pthread)
endif()
