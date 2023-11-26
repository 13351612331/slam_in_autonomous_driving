cmake_minimum_required(VERSION 3.0)
project(CH_4)

include_directories(../ch3
        ../../common
        ../../thirdParty/g2o)

list(APPEND CH4_SRCS imu_preintegration.cpp g2o_types.cpp)
list(APPEND CH3_SRCS ../ch3/utm_convert.cpp ../ch3/static_imu_init.cpp)
file(GLOB g2o_LIBS ../../thirdParty/g2o/lib/*.so)

find_package(GTest REQUIRED)
message(${GTEST_LIBRARIES})

add_executable(test_preintegraion test_preintegration.cpp ${CH4_SRCS} ${CH3_SRCS} ${COMMON_SRCS})
target_link_libraries(test_preintegraion ${GTEST_LIBRARIES} ${ALL_LIBS} ${g2o_LIBS})