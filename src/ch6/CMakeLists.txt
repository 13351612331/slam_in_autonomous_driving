cmake_minimum_required(VERSION 3.0)
project(CH_6)

find_package(OpenCV REQUIRED)

list(APPEND CH6_SRCS lidar_2d_utils.cpp icp_2d.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_2dlidar_io test_2dlidar_io.cpp ${CH6_SRCS} ${COMMON_SRCS})
target_link_libraries(test_2dlidar_io ${ALL_LIBS} ${OpenCV_LIBS})

add_executable(test_2d_icp_s2s test_2d_icp_s2s.cpp ${CH6_SRCS} ${COMMON_SRCS})
target_link_libraries(test_2d_icp_s2s ${ALL_LIBS} ${OpenCV_LIBS})
