cmake_minimum_required(VERSION 3.0)
project(SLAM_IN_AUTONOMOUS_DRIVING)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(USE_EIGEN)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(gflags REQUIRED)
find_package(glog REQUIRED)

include_directories(thirdParty/sophus
        thirdParty/utm_convert
        thirdParty/g2o
)

message("=================================")
message(${Pangolin_LIBRARIES})

list(APPEND ALL_LIBS ${PCL_LIBRARY_DIRS})
list(APPEND ALL_LIBS ${Pangolin_LIBRARIES})
list(APPEND ALL_LIBS ${gflags_LIBRARIES})
list(APPEND ALL_LIBS glog::glog)
list(APPEND ALL_LIBS tbb)

file(GLOB_RECURSE COMMON_SRCS common/*.cpp thirdParty/utm_convert/*.cc)

add_subdirectory(src)