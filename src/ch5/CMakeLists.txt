cmake_minimum_required(VERSION 3.0)
project(CH_5)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTest REQUIRED)

list(APPEND CH5_SRCS bfnn.cpp kdtree.cpp octo_tree.cpp)

file(GLOB g2o_LIBS ../../thirdParty/g2o/lib/*.so)
list(APPEND ALL_LIBS "${g2o_LIBS}")


add_executable(point_cloud_load_and_vis point_cloud_load_and_vis.cpp)
target_link_libraries(point_cloud_load_and_vis ${ALL_LIBS} ${PCL_LIBRARIES})

add_executable(pcd_to_bird_eye pcd_to_bird_eye.cpp)
target_link_libraries(pcd_to_bird_eye ${ALL_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(scan_to_range_image scan_to_range_image.cpp)
target_link_libraries(scan_to_range_image ${ALL_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_nn test_nn.cpp ${CH5_SRCS} ${COMMON_SRCS})
target_link_libraries(test_nn ${ALL_LIBS} ${GTEST_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(linear_fitting linear_fitting.cpp ${CH5_SRCS} ${COMMON_SRCS})
target_link_libraries(linear_fitting ${ALL_LIBS} ${GTEST_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})