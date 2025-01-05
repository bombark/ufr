add_executable(test_gtw_qt ./main.c)
target_link_libraries(test_gtw_qt ufr_gtw_qt)
add_test(NAME test_gtw_qt COMMAND test_gtw_qt)
