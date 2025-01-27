add_executable(test_enc_http ./test_enc_http.c)
target_link_libraries(test_enc_http ufr_enc_http)
add_test(NAME test_enc_http COMMAND test_enc_http)

add_executable(test_dcr_http ./test_dcr_http.c)
target_link_libraries(test_dcr_http ufr_dcr_http)
add_test(NAME test_dcr_http COMMAND test_dcr_http)