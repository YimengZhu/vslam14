cmake_minimum_required(VERSION 3.5)

project(disparity)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(disparity disparity.cpp)

target_link_libraries(disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBRARIES})
