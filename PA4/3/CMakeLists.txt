cmake_minimum_required(VERSION 3.5)

project(gn)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(gn gaussnewton.cpp)

target_link_libraries(gn ${OpenCV_LIBRARIES})
