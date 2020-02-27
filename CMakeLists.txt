cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_library(serialComm lib/serialcomm/serialComm.c)
add_executable(simplertk2b main.cpp)

add_compile_options(-std=gnu99)