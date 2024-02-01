include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test1 ./test1.cpp)
target_link_libraries(test1 ufr ufr_gtw_ros_humble ufr_ecr_ros_humble ufr_dcr_ros_humble  )
add_test(NAME test1 COMMAND test1)