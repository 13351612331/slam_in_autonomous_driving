cmake_minimum_required(VERSION 3.0)
project(CH_3)

list(APPEND CH3_SRC utm_convert.cpp static_imu_init.cpp)

file(GLOB g2o_LIBS ../../thirdParty/g2o/lib/*.so)

get_filename_component(full_path_test_cpp ${CMAKE_CURRENT_SOURCE_DIR}/../../common/point_cloud_utils.cpp ABSOLUTE)

#add_library(COMMON_LIBS SHARED ${COMMON_SRCS})
LIST(REMOVE_ITEM COMMON_SRCS "${full_path_test_cpp}")

add_executable(run_imu_integration run_imu_integration.cpp
        ${UI_SRCS} ${COMMON_SRCS} ${CH3_SRC})

target_link_libraries(run_imu_integration ${ALL_LIBS} ${g2o_LIBS})

add_executable(process_gnss process_gnss.cpp
        ${UI_SRCS} ${COMMON_SRCS} ${CH3_SRC})

target_link_libraries(process_gnss ${ALL_LIBS} ${g2o_LIBS})

add_executable(run_eskf_gins run_eskf_gins.cpp ${UI_SRCS} ${COMMON_SRCS} ${CH3_SRC})
target_link_libraries(run_eskf_gins ${ALL_LIBS} ${g2o_LIBS})