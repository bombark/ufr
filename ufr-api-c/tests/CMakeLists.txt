add_executable(test_api ./test_api.c)
target_link_libraries(test_api ufr)
add_test(NAME test_api COMMAND test_api)

add_executable(test_std_dcr ./test_std_dcr.c)
target_link_libraries(test_std_dcr ufr)
add_test(NAME test_std_dcr COMMAND test_std_dcr)

add_executable(test_std_enc ./test_std_enc.c)
target_link_libraries(test_std_enc ufr)
add_test(NAME test_std_enc COMMAND test_std_enc)

add_executable(test_buffer ./test_buffer.c)
target_link_libraries(test_buffer ufr)
add_test(NAME test_buffer COMMAND test_buffer)