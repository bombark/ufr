add_executable(test_gtw_zmq ./test_gtw_zmq.c)
target_link_libraries(test_gtw_zmq ufr_gtw_zmq pthread)
add_test(NAME test_gtw_zmq COMMAND test_gtw_zmq)
