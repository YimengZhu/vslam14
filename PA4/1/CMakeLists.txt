cmake_minimum_required(VERSION 3.5)

project(undistorted)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistorted undistort_image.cpp)

target_link_libraries(undistorted ${OpenCV_LIBRARIES})
