cmake_minimum_required(VERSION 3.0)
project(CH_3)

list(APPEND CH3_SRC utm_convert.cpp)

add_executable(run_imu_integration run_imu_integration.cpp
        ${UI_SRCS} ${COMMON_SRCS} ${CH3_SRC})

target_link_libraries(run_imu_integration ${ALL_LIBS})

add_executable(process_gnss process_gnss.cpp
        ${UI_SRCS} ${COMMON_SRCS} ${CH3_SRC})

target_link_libraries(process_gnss ${ALL_LIBS})