include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test1 ./test1.cpp)
target_link_libraries(test1 ufr_gtw_ros_noetic ufr)
# add_test(NAME test1 COMMAND test1)
# install(TARGETS test1 DESTINATION bin)