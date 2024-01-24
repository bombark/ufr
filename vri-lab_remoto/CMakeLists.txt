cmake_minimum_required(VERSION 3.0)
project(vri-slam-lama)

include_directories( ${EIGEN3_INCLUDE_DIR} )
find_package(OpenCV REQUIRED)

add_executable(vri-lab_remoto-camera src/camera.cpp)
target_link_libraries(vri-lab_remoto-camera ${OpenCV_LIBS} ufr)

add_executable(teste src/camera_sub.cpp)
target_link_libraries(teste ${OpenCV_LIBS} ufr)



# install(TARGETS vri-slam DESTINATION bin)