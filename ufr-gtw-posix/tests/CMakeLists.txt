add_executable(test_posix_file ./test_file.c)
target_link_libraries(test_posix_file ufr_gtw_posix)
add_test(NAME test_posix_file COMMAND test_posix_file)

add_executable(test_posix_pipe ./test_pipe.c)
target_link_libraries(test_posix_pipe ufr_gtw_posix)
add_test(NAME test_posix_pipe COMMAND test_posix_pipe)

add_executable(test_posix_socket ./test_socket.c)
target_link_libraries(test_posix_socket ufr_gtw_posix)
add_test(NAME test_posix_socket COMMAND test_posix_socket)