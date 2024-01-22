add_executable(test_gw_httpd ./main.c)
target_link_libraries(test_gw_httpd gw_http)
add_test(NAME test_gw_httpd COMMAND test_gw_httpd)