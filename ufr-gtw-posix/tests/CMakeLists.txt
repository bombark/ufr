add_executable(test_file ./test_file.c)
target_link_libraries(test_file ufr_gtw_posix)
add_test(NAME test_file COMMAND test_file)

add_executable(test_pipe ./test_pipe.c)
target_link_libraries(test_pipe ufr_gtw_posix)
add_test(NAME test_pipe COMMAND test_pipe)
