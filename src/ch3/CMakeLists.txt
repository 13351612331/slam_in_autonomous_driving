cmake_minimum_required(VERSION 3.0)
project(CH_3)

add_executable(run_imu_integration run_imu_integration.cpp
        ${UI_SRCS} ${COMMON_SRCS})

target_link_libraries(run_imu_integration ${ALL_LIBS})