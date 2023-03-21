cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
)

include(cmake/eigen.cmake)
include(cmake/pcl.cmake)
file(GLOB_RECURSE ALL_SRCS "src/*.c*")

add_executable(triad node/triad.cpp ${ALL_SRCS})
target_link_libraries(triad ${ALL_TARGET_LIBRARIES})

add_executable(bun_test node/bun_test.cpp ${ALL_SRCS})
target_link_libraries(bun_test ${ALL_TARGET_LIBRARIES})

