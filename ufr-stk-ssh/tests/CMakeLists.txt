add_executable(ssh_test ./test1.c)
target_link_libraries(ssh_test ufr ufr_gtw_ssh)
add_test(NAME ssh_test COMMAND ssh_test)