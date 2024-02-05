cmake_minimum_required(VERSION 3.0)
project(ufr-tests)

add_executable(pub_ii ./src/pub_ii.c)
target_link_libraries(pub_ii ufr)

add_executable(sub_ii ./src/sub_ii.c)
target_link_libraries(sub_ii ufr)

add_executable(testlib ./src/testlib.c)
target_link_libraries(testlib ufr dl)