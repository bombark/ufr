add_executable(test_api ./test_api.c)
target_link_libraries(test_api ufr_gtw_zmq)
add_test(NAME test_api COMMAND test_api)
