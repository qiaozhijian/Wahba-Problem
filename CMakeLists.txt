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
file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_executable(shuster_solution node/shuster_solution.cpp ${ALL_SRCS})
target_link_libraries(shuster_solution ${ALL_TARGET_LIBRARIES})

