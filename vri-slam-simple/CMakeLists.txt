cmake_minimum_required(VERSION 3.0)
project(vri-slam-simple)

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package(OpenCV REQUIRED)

add_executable(vri-slam 
    ./src/main.cpp 
    )

target_link_libraries(vri-slam ufr ${OpenCV_LIBS})

install(TARGETS vri-slam DESTINATION bin)