add_executable(test_gtw_gtk ./main.c)
target_link_libraries(test_gtw_gtk ufr_gtw_gtk)
add_test(NAME test_gtw_gtk COMMAND test_gtw_gtk)
