add_executable(webots_test ./test1.c)
target_link_libraries(webots_test ufr ufr_gtw_webots m)
add_test(NAME webots_test COMMAND webots_test)