cmake_minimum_required(VERSION 3.5)
project(directBA)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED )
find_package(Sophus REQUIRED)
find_package( G2O REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(Sophus_LIBS libSophus.so)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} directBA.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} 
					${Sophus_LIBS} 
					${EIGEN3_LIBS} 
					${Pangolin_LIBRARIES}
				       g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)	
