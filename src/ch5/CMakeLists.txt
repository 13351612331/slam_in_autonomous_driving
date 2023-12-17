cmake_minimum_required(VERSION 3.0)
project(CH_5)

find_package(PCL REQUIRED)


add_executable(point_cloud_load_and_vis point_cloud_load_and_vis.cpp)
target_link_libraries(point_cloud_load_and_vis ${ALL_LIBS} ${PCL_LIBRARIES})