cmake_minimum_required(VERSION 3.0)
project(CH_5)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(point_cloud_load_and_vis point_cloud_load_and_vis.cpp)
target_link_libraries(point_cloud_load_and_vis ${ALL_LIBS} ${PCL_LIBRARIES})

add_executable(pcd_to_bird_eye pcd_to_bird_eye.cpp)
target_link_libraries(pcd_to_bird_eye ${ALL_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})