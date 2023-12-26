cmake_minimum_required(VERSION 3.0)
project(CH_4)

include_directories(../ch3
        ../../common
        ../../thirdParty/g2o)

list(APPEND CH4_SRCS imu_preintegration.cpp g2o_types.cpp gins_pre_integ.cpp)
list(APPEND CH3_SRCS ../ch3/utm_convert.cpp ../ch3/static_imu_init.cpp)
file(GLOB g2o_LIBS ../../thirdParty/g2o/lib/*.so)

find_package(GTest REQUIRED)
message(${GTEST_LIBRARIES})

get_filename_component(full_path_test_cpp ${CMAKE_CURRENT_SOURCE_DIR}/../../common/point_cloud_utils.cpp ABSOLUTE)

#add_library(COMMON_LIBS SHARED ${COMMON_SRCS})
LIST(REMOVE_ITEM COMMON_SRCS "${full_path_test_cpp}")


add_executable(test_preintegraion test_preintegration.cpp ${UI_SRCS} ${COMMON_SRCS} ${CH4_SRCS} ${CH3_SRCS})
target_link_libraries(test_preintegraion ${GTEST_LIBRARIES} ${ALL_LIBS} ${g2o_LIBS})

add_executable(run_gins_pre_integ run_gins_pre_integ.cpp ${UI_SRCS} ${COMMON_SRCS} ${CH4_SRCS} ${CH3_SRCS})
target_link_libraries(run_gins_pre_integ ${GTEST_LIBRARIES} ${ALL_LIBS} ${g2o_LIBS})