add_executable(test_gtw_sqlite ./test_gtw_sqlite.c)
target_link_libraries(test_gtw_sqlite ufr_gtw_sqlite)
add_test(NAME test_gtw_sqlite COMMAND test_gtw_sqlite)
