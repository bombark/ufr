cmake_minimum_required(VERSION 3.5)
project(ufr-tests)

# add_executable(testlib ./src/testlib.c)
# target_link_libraries(testlib ufr dl)
# install(TARGETS testlib DESTINATION tests)

add_executable(test_topic ./src/test_topic.c)
target_link_libraries(test_topic ufr dl)
install(TARGETS test_topic DESTINATION tests)

add_executable(test_socket ./src/test_socket.c)
target_link_libraries(test_socket ufr dl)
install(TARGETS test_socket DESTINATION tests)