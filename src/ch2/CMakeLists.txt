cmake_minimum_required(VERSION 3.0)
project(CH_2)

add_executable(motion motion.cpp
        ${UI_SRCS})

target_link_libraries(motion ${ALL_LIBS})