add_executable(test_gtw_httpd ./main.c)
target_link_libraries(test_gtw_httpd gtw_http)
add_test(NAME test_gtw_httpd COMMAND test_gtw_httpd)