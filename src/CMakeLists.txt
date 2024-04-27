cmake_minimum_required(VERSION 3.2)
project(ProjectName)
file(GLOB_RECURSE UI_SRCS tools/ui/*.cpp)
include_directories(
        ../
        tools/ui)
add_subdirectory(ch2)
add_subdirectory(ch3)
add_subdirectory(ch4)
add_subdirectory(ch5)
add_subdirectory(ch6)