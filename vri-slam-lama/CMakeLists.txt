cmake_minimum_required(VERSION 3.0)
project(vri-slam-lama)

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )
# find_package(OpenCV REQUIRED)

add_executable(vri-slam src/main.cpp)
target_link_libraries(vri-slam iris_lama)

install(TARGETS vri-slam DESTINATION bin)